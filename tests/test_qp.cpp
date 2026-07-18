#include "helix/osqp_solver.hpp"

#include "test_utils.hpp"

#include <Eigen/Sparse>

#include <limits>
#include <vector>

namespace {

helix::QpProblem unconstrained_problem(double first_linear_term = -1.0) {
    helix::QpProblem problem;
    Eigen::Matrix2d h;
    h << 4.0, 1.0, 1.0, 2.0;
    problem.H = h.sparseView();
    problem.c.resize(2);
    problem.c << first_linear_term, -1.0;
    problem.A.resize(0, 2);
    problem.lower.resize(0);
    problem.upper.resize(0);
    return problem;
}

}  // namespace

int main() {
    helix::OsqpSolver solver;

    // Full, non-diagonal symmetric H must be accepted. No simplex constraint is implicit.
    const helix::SolveResult first = solver.solve(unconstrained_problem());
    TEST_REQUIRE(first.success());
    TEST_REQUIRE(near(first.x[0], 1.0 / 7.0));
    TEST_REQUIRE(near(first.x[1], 3.0 / 7.0));
    TEST_REQUIRE(!first.stats.reused_workspace);

    // Same sparsity pattern updates data in-place and retains the workspace.
    const helix::SolveResult second = solver.solve(unconstrained_problem(-2.0));
    TEST_REQUIRE(second.success());
    TEST_REQUIRE(second.stats.reused_workspace);
    TEST_REQUIRE(!second.stats.updated_matrices);
    TEST_REQUIRE(near(second.x[0], 3.0 / 7.0));
    TEST_REQUIRE(near(second.x[1], 2.0 / 7.0));

    // Contradictory inequalities are reported as infeasible, not as a zero solution.
    helix::QpProblem infeasible;
    infeasible.H.resize(1, 1);
    infeasible.H.insert(0, 0) = 1.0;
    infeasible.c = Eigen::VectorXd::Zero(1);
    infeasible.A.resize(2, 1);
    infeasible.A.insert(0, 0) = 1.0;
    infeasible.A.insert(1, 0) = 1.0;
    infeasible.lower.resize(2);
    infeasible.upper.resize(2);
    infeasible.lower << 1.0, -std::numeric_limits<double>::infinity();
    infeasible.upper << std::numeric_limits<double>::infinity(), 0.0;
    const helix::SolveResult infeasible_result = solver.solve(infeasible);
    TEST_REQUIRE(infeasible_result.status == helix::SolveStatus::kPrimalInfeasible);
    TEST_REQUIRE(infeasible_result.x.size() == 0);

    helix::QpProblem invalid = unconstrained_problem();
    invalid.H.coeffRef(1, 0) = 9.0;
    const helix::SolveResult invalid_result = solver.solve(invalid);
    TEST_REQUIRE(invalid_result.status == helix::SolveStatus::kInvalidProblem);
    TEST_REQUIRE(!invalid_result.message.empty());
    return 0;
}
