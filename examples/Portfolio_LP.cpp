#include "helix/lp_solver.hpp"

#include "portfolio_common.hpp"

#include <Eigen/Dense>

#include <exception>
#include <iostream>

int main() {
    const helix_examples::PortfolioInput input = helix_examples::make_example_input();

    helix::SolverSettings settings;
    settings.absolute_tolerance = 1e-7;
    settings.relative_tolerance = 1e-7;
    settings.polish = true;
    helix::LpSolver solver(settings);

    try {
        const helix_examples::PortfolioConstraints constraints =
            helix_examples::make_portfolio_constraints(input);

        helix::LpProblem problem;
        problem.c = Eigen::VectorXd::Zero(2 * input.alpha.size());
        problem.c.head(input.alpha.size()) = -input.alpha;
        problem.A = constraints.matrix;
        problem.lower = constraints.lower;
        problem.upper = constraints.upper;

        helix_examples::PortfolioResult result =
            helix_examples::decode_result(input, solver.solve(problem));
        if (!result.solve_result.success()) {
            std::cerr << "portfolio LP failed: " << result.solve_result.message << '\n';
            return 1;
        }
        helix_examples::print_result(input, result);
    } catch (const std::exception& error) {
        std::cerr << "invalid portfolio input: " << error.what() << '\n';
        return 2;
    }
    return 0;
}
