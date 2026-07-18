#pragma once

#include "helix/types.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace helix_examples {

struct PortfolioInput {
    Eigen::VectorXd alpha;
    Eigen::VectorXd current_position_value;
    Eigen::VectorXd max_position_weight;
    Eigen::VectorXd liquidity_limit_value;
    double max_turnover_ratio{0.0};
    double capital{0.0};
};

struct PortfolioResult {
    Eigen::VectorXd target_position_value;
    Eigen::VectorXd target_weight;
    Eigen::VectorXd trade_value;
    double expected_alpha{0.0};
    double turnover_ratio{0.0};
    helix::SolveResult solve_result;
};

struct PortfolioConstraints {
    helix::SparseMatrix matrix;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

inline void validate_input(const PortfolioInput& input) {
    const Eigen::Index assets = input.alpha.size();
    if (assets == 0 || input.current_position_value.size() != assets ||
        input.max_position_weight.size() != assets ||
        input.liquidity_limit_value.size() != assets) {
        throw std::invalid_argument("all portfolio vectors must have the same non-zero size");
    }
    if (!input.alpha.allFinite() || !input.current_position_value.allFinite() ||
        !input.max_position_weight.allFinite() || !input.liquidity_limit_value.allFinite()) {
        throw std::invalid_argument("portfolio inputs must be finite");
    }
    if (!std::isfinite(input.capital) || input.capital <= 0.0) {
        throw std::invalid_argument("capital must be finite and positive");
    }
    if (!std::isfinite(input.max_turnover_ratio) || input.max_turnover_ratio < 0.0) {
        throw std::invalid_argument("max_turnover_ratio must be finite and non-negative");
    }
    if ((input.current_position_value.array() < 0.0).any() ||
        (input.max_position_weight.array() < 0.0).any() ||
        (input.liquidity_limit_value.array() < 0.0).any()) {
        throw std::invalid_argument(
            "positions, position limits and liquidity must be non-negative");
    }

    const double capital_tolerance = 1e-8 * std::max(1.0, input.capital);
    if (std::abs(input.current_position_value.sum() - input.capital) > capital_tolerance) {
        throw std::invalid_argument("current positions must sum to capital");
    }
}

inline PortfolioConstraints make_portfolio_constraints(const PortfolioInput& input) {
    validate_input(input);

    const Eigen::Index assets = input.alpha.size();
    const Eigen::Index variables = 2 * assets;  // [target weights w, absolute trades t]
    const Eigen::Index constraint_count = 4 * assets + 2;
    const double infinity = std::numeric_limits<double>::infinity();

    PortfolioConstraints constraints;
    constraints.matrix.resize(constraint_count, variables);
    constraints.lower = Eigen::VectorXd::Constant(constraint_count, -infinity);
    constraints.upper = Eigen::VectorXd::Constant(constraint_count, infinity);

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(static_cast<std::size_t>(8 * assets));

    // Fully invested: sum(w) = 1.
    for (Eigen::Index asset = 0; asset < assets; ++asset) {
        entries.emplace_back(0, asset, 1.0);
    }
    constraints.lower[0] = 1.0;
    constraints.upper[0] = 1.0;

    const Eigen::Index position_bound_row = 1;
    const Eigen::Index trade_bound_row = position_bound_row + assets;
    const Eigen::Index positive_trade_row = trade_bound_row + assets;
    const Eigen::Index negative_trade_row = positive_trade_row + assets;
    const Eigen::Index total_turnover_row = negative_trade_row + assets;

    double reachable_lower_sum = 0.0;
    double reachable_upper_sum = 0.0;
    for (Eigen::Index asset = 0; asset < assets; ++asset) {
        const Eigen::Index target_weight = asset;
        const Eigen::Index absolute_trade_weight = assets + asset;
        const double current_weight = input.current_position_value[asset] / input.capital;
        const double liquidity_weight = input.liquidity_limit_value[asset] / input.capital;

        const double position_lower = std::max(0.0, current_weight - liquidity_weight);
        const double position_upper =
            std::min(input.max_position_weight[asset], current_weight + liquidity_weight);
        if (position_lower > position_upper) {
            throw std::invalid_argument("position and liquidity limits are mutually infeasible");
        }
        reachable_lower_sum += position_lower;
        reachable_upper_sum += position_upper;

        entries.emplace_back(position_bound_row + asset, target_weight, 1.0);
        constraints.lower[position_bound_row + asset] = position_lower;
        constraints.upper[position_bound_row + asset] = position_upper;

        entries.emplace_back(trade_bound_row + asset, absolute_trade_weight, 1.0);
        constraints.lower[trade_bound_row + asset] = 0.0;
        constraints.upper[trade_bound_row + asset] = liquidity_weight;

        // t_i >= |w_i - current_weight_i|.
        entries.emplace_back(positive_trade_row + asset, target_weight, 1.0);
        entries.emplace_back(positive_trade_row + asset, absolute_trade_weight, -1.0);
        constraints.upper[positive_trade_row + asset] = current_weight;

        entries.emplace_back(negative_trade_row + asset, target_weight, -1.0);
        entries.emplace_back(negative_trade_row + asset, absolute_trade_weight, -1.0);
        constraints.upper[negative_trade_row + asset] = -current_weight;

        entries.emplace_back(total_turnover_row, absolute_trade_weight, 1.0);
    }

    constexpr double feasibility_tolerance = 1e-10;
    if (1.0 < reachable_lower_sum - feasibility_tolerance ||
        1.0 > reachable_upper_sum + feasibility_tolerance) {
        throw std::invalid_argument(
            "capital cannot be reached under position and liquidity limits");
    }

    // One-way turnover = sum(abs(target - current)) / (2 * capital).
    constraints.upper[total_turnover_row] = 2.0 * input.max_turnover_ratio;
    constraints.matrix.setFromTriplets(entries.begin(), entries.end());
    return constraints;
}

inline PortfolioResult decode_result(const PortfolioInput& input,
                                     helix::SolveResult solve_result) {
    PortfolioResult result;
    result.solve_result = std::move(solve_result);
    if (!result.solve_result.success()) {
        return result;
    }

    const Eigen::Index assets = input.alpha.size();
    result.target_weight = result.solve_result.x.head(assets);
    result.target_position_value = result.target_weight * input.capital;
    result.trade_value = result.target_position_value - input.current_position_value;
    result.turnover_ratio = result.trade_value.cwiseAbs().sum() / (2.0 * input.capital);
    result.expected_alpha = input.alpha.dot(result.target_weight);
    return result;
}

inline void print_result(const PortfolioInput& input, const PortfolioResult& result) {
    std::cout << std::fixed << std::setprecision(6)
              << "status: " << helix::to_string(result.solve_result.status) << '\n'
              << "capital: " << input.capital << '\n'
              << "target value: " << result.target_position_value.transpose() << '\n'
              << "target weight: " << result.target_weight.transpose() << '\n'
              << "trade value: " << result.trade_value.transpose() << '\n'
              << "expected alpha: " << result.expected_alpha << '\n'
              << "turnover ratio: " << result.turnover_ratio << '\n'
              << "turnover limit: " << input.max_turnover_ratio << '\n'
              << "iterations: " << result.solve_result.stats.iterations << '\n'
              << "primal residual: " << result.solve_result.stats.primal_residual << '\n';
}

inline PortfolioInput make_example_input() {
    PortfolioInput input;
    input.alpha.resize(4);
    input.alpha << 0.02, 0.08, 0.05, -0.01;
    input.current_position_value.resize(4);
    input.current_position_value << 300'000.0, 250'000.0, 250'000.0, 200'000.0;
    input.max_position_weight.resize(4);
    input.max_position_weight << 0.35, 0.40, 0.30, 0.25;
    input.liquidity_limit_value.resize(4);
    input.liquidity_limit_value << 100'000.0, 120'000.0, 80'000.0, 60'000.0;
    input.max_turnover_ratio = 0.10;
    input.capital = 1'000'000.0;
    return input;
}

}  // namespace helix_examples
