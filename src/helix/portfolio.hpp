#pragma once

#include "lp_solver.hpp"
#include "types.hpp"

namespace helix {

// Inputs for a long-only, fully-invested portfolio optimization.
// current_position_value, liquidity_limit_value and capital must use the same currency unit.
struct PortfolioOptimizationInput {
    Eigen::VectorXd alpha;
    Eigen::VectorXd current_position_value;
    Eigen::VectorXd max_position_weight;
    Eigen::VectorXd liquidity_limit_value;
    double max_turnover_ratio{0.0};
    double capital{0.0};
};

struct PortfolioOptimizationResult {
    Eigen::VectorXd target_position_value;
    Eigen::VectorXd target_weight;
    Eigen::VectorXd trade_value;
    double expected_alpha{0.0};
    double turnover_ratio{0.0};
    SolveResult solver_result;

    [[nodiscard]] bool success() const noexcept { return solver_result.success(); }
};

// Reusable linear constraint system for variables [target_weight, absolute_trade_weight].
// Exposed so portfolio QP models can share exactly the same trading constraints.
struct PortfolioConstraintSystem {
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

// Builds these constraints:
//   sum(weight) = 1, weight >= 0, position caps, per-name liquidity limits, and
//   one-way turnover = sum(abs(target-current)) / (2*capital) <= max_turnover_ratio.
// Throws std::invalid_argument when the input is malformed or trivially infeasible.
PortfolioConstraintSystem make_portfolio_constraints(
    const PortfolioOptimizationInput& input);

// Converts the first n solver variables (target weights) into portfolio-domain outputs.
PortfolioOptimizationResult make_portfolio_result(
    const PortfolioOptimizationInput& input, SolveResult solve_result);

class PortfolioLpOptimizer {
public:
    explicit PortfolioLpOptimizer(SolverSettings settings = {});

    PortfolioOptimizationResult solve(const PortfolioOptimizationInput& input);
    void reset() noexcept;

private:
    LpSolver solver_;
};

// Convenience entry point for one-off optimization. Prefer PortfolioLpOptimizer when repeatedly
// updating alpha or limits, so the OSQP workspace and warm start can be reused.
PortfolioOptimizationResult optimize_portfolio_lp(
    const PortfolioOptimizationInput& input, SolverSettings settings = {});

}  // namespace helix
