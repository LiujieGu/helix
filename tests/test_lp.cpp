#include "helix/lp_solver.hpp"

#include "test_utils.hpp"

#include <Eigen/Sparse>

#include <limits>
#include <vector>

int main() {
    helix::LpProblem problem;
    problem.c.resize(2);
    problem.c << -1.0, -2.0;
    problem.A.resize(3, 2);
    const std::vector<Eigen::Triplet<double>> entries{
        {0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {2, 1, 1.0}};
    problem.A.setFromTriplets(entries.begin(), entries.end());
    problem.lower.resize(3);
    problem.upper.resize(3);
    problem.lower << 1.0, 0.0, 0.0;
    problem.upper << 1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity();

    helix::LpSolver solver;
    const helix::SolveResult result = solver.solve(problem);
    TEST_REQUIRE(result.success());
    TEST_REQUIRE(result.x.size() == 2);
    TEST_REQUIRE(near(result.x.sum(), 1.0));
    TEST_REQUIRE(result.x[0] >= -1e-5);
    TEST_REQUIRE(result.x[1] >= 1.0 - 1e-5);
    TEST_REQUIRE(result.stats.primal_residual <= 1e-5);
    return 0;
}
