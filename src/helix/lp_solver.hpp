#pragma once

#include "osqp_solver.hpp"

namespace helix {

// General convex LP backend. For the special probability-simplex LP, use
// minimize_linear_on_simplex() for an exact O(n) solution.
class LpSolver {
public:
    explicit LpSolver(SolverSettings settings = {}) : backend_(settings) {}

    SolveResult solve(const LpProblem& problem) {
        QpProblem qp;
        qp.H.resize(problem.c.size(), problem.c.size());
        qp.c = problem.c;
        qp.A = problem.A;
        qp.lower = problem.lower;
        qp.upper = problem.upper;
        return backend_.solve(qp);
    }

    [[nodiscard]] const char* name() const noexcept { return "LP (OSQP backend)"; }
    void reset() noexcept { backend_.reset(); }

private:
    OsqpSolver backend_;
};

}  // namespace helix
