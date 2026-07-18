#include "helix/portfolio.hpp"

#include "portfolio_common.hpp"

#include <iostream>

int main() {
    const helix::PortfolioOptimizationInput input = helix_examples::make_example_input();

    helix::SolverSettings settings;
    settings.absolute_tolerance = 1e-7;
    settings.relative_tolerance = 1e-7;
    settings.polish = true;

    // Keep this object alive across calls to reuse the OSQP workspace and warm start.
    helix::PortfolioLpOptimizer optimizer(settings);
    const helix::PortfolioOptimizationResult result = optimizer.solve(input);
    if (!result.success()) {
        std::cerr << "portfolio LP failed: " << result.solver_result.message << '\n';
        return 1;
    }

    helix_examples::print_result(input, result);
    return 0;
}
