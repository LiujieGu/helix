#pragma once

#include "helix/portfolio.hpp"

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>

namespace helix_examples {

inline void print_result(const helix::PortfolioOptimizationInput& input,
                         const helix::PortfolioOptimizationResult& result) {
    std::cout << std::fixed << std::setprecision(6)
              << "status: " << helix::to_string(result.solver_result.status) << '\n'
              << "capital: " << input.capital << '\n'
              << "target value: " << result.target_position_value.transpose() << '\n'
              << "target weight: " << result.target_weight.transpose() << '\n'
              << "trade value: " << result.trade_value.transpose() << '\n'
              << "expected alpha: " << result.expected_alpha << '\n'
              << "turnover ratio: " << result.turnover_ratio << '\n'
              << "turnover limit: " << input.max_turnover_ratio << '\n'
              << "iterations: " << result.solver_result.stats.iterations << '\n'
              << "primal residual: " << result.solver_result.stats.primal_residual << '\n';
}

inline helix::PortfolioOptimizationInput make_example_input() {
    helix::PortfolioOptimizationInput input;
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
