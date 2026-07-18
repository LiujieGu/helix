#include "helix/portfolio.hpp"

#include "test_utils.hpp"

namespace {

helix::PortfolioOptimizationInput make_input() {
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

}  // namespace

int main() {
    helix::SolverSettings settings;
    settings.absolute_tolerance = 1e-7;
    settings.relative_tolerance = 1e-7;
    helix::PortfolioLpOptimizer optimizer(settings);

    helix::PortfolioOptimizationInput input = make_input();
    const helix::PortfolioOptimizationResult first = optimizer.solve(input);
    TEST_REQUIRE(first.success());
    TEST_REQUIRE(near(first.target_weight.sum(), 1.0));
    TEST_REQUIRE(near(first.target_position_value.sum(), input.capital, 1e-2));
    TEST_REQUIRE(first.target_weight.minCoeff() >= -1e-7);
    TEST_REQUIRE(first.turnover_ratio <= input.max_turnover_ratio + 1e-7);
    TEST_REQUIRE(near(first.target_weight[0], 0.26));
    TEST_REQUIRE(near(first.target_weight[1], 0.35));
    TEST_REQUIRE(near(first.target_weight[2], 0.25));
    TEST_REQUIRE(near(first.target_weight[3], 0.14));

    input.alpha[2] += 0.01;
    const helix::PortfolioOptimizationResult updated = optimizer.solve(input);
    TEST_REQUIRE(updated.success());
    TEST_REQUIRE(updated.solver_result.stats.reused_workspace);
    TEST_REQUIRE(!updated.solver_result.stats.updated_matrices);

    input.capital = -1.0;
    const helix::PortfolioOptimizationResult invalid = optimizer.solve(input);
    TEST_REQUIRE(invalid.solver_result.status == helix::SolveStatus::kInvalidProblem);
    TEST_REQUIRE(!invalid.solver_result.message.empty());
    return 0;
}
