#include "portfolio.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helix {
namespace {

void validate_input(const PortfolioOptimizationInput& input) {
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

}  // namespace

PortfolioOptimizationResult make_portfolio_result(
    const PortfolioOptimizationInput& input, SolveResult solve_result) {
    PortfolioOptimizationResult result;
    result.solver_result = std::move(solve_result);
    if (!result.success()) {
        return result;
    }

    const Eigen::Index assets = input.alpha.size();
    result.target_weight = result.solver_result.x.head(assets);
    result.target_position_value = result.target_weight * input.capital;
    result.trade_value = result.target_position_value - input.current_position_value;
    result.turnover_ratio = result.trade_value.cwiseAbs().sum() / (2.0 * input.capital);
    result.expected_alpha = input.alpha.dot(result.target_weight);
    return result;
}

PortfolioConstraintSystem make_portfolio_constraints(
    const PortfolioOptimizationInput& input) {
    validate_input(input);

    const Eigen::Index assets = input.alpha.size();
    const Eigen::Index variables = 2 * assets;
    const Eigen::Index constraint_count = 4 * assets + 2;
    const double infinity = std::numeric_limits<double>::infinity();

    PortfolioConstraintSystem constraints;
    constraints.A.resize(constraint_count, variables);
    constraints.lower = Eigen::VectorXd::Constant(constraint_count, -infinity);
    constraints.upper = Eigen::VectorXd::Constant(constraint_count, infinity);

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(static_cast<std::size_t>(8 * assets));

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

    constraints.upper[total_turnover_row] = 2.0 * input.max_turnover_ratio;
    constraints.A.setFromTriplets(entries.begin(), entries.end());
    return constraints;
}

PortfolioLpOptimizer::PortfolioLpOptimizer(SolverSettings settings) : solver_(settings) {}

PortfolioOptimizationResult PortfolioLpOptimizer::solve(
    const PortfolioOptimizationInput& input) {
    try {
        PortfolioConstraintSystem constraints = make_portfolio_constraints(input);
        LpProblem problem;
        problem.c = Eigen::VectorXd::Zero(2 * input.alpha.size());
        problem.c.head(input.alpha.size()) = -input.alpha;
        problem.A = std::move(constraints.A);
        problem.lower = std::move(constraints.lower);
        problem.upper = std::move(constraints.upper);
        return make_portfolio_result(input, solver_.solve(problem));
    } catch (const std::invalid_argument& error) {
        PortfolioOptimizationResult result;
        result.solver_result.status = SolveStatus::kInvalidProblem;
        result.solver_result.message = error.what();
        return result;
    }
}

void PortfolioLpOptimizer::reset() noexcept {
    solver_.reset();
}

PortfolioOptimizationResult optimize_portfolio_lp(
    const PortfolioOptimizationInput& input, SolverSettings settings) {
    PortfolioLpOptimizer optimizer(settings);
    return optimizer.solve(input);
}

}  // namespace helix
