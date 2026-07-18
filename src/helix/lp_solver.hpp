/// Solves LP:  maximize  signal^T x  subject to  sum(x)=1, x >= 0
/// Converts to QP form: minimize  -signal^T x  with same constraints.
///
/// Algorithm: projected gradient ascent on the probability simplex.
/// Uses sorted-simplex projection (Duchi et al., 2008).
///
/// The "problem.c" stores -signal (since we minimize -signal^T x,
/// i.e. maximize signal^T x).  problem.H must be zero.
#pragma once

#include "solver.hpp"
#include "simplex.hpp"

namespace helix {

class LpSolver : public Solver {
public:
    explicit LpSolver(double step = 0.1, int max_iters = 2000, double tol = 1e-6)
        : step_signal_(step), max_iters_(max_iters), tol_(tol) {}

    QpResult solve(const QpProblem& problem) override {
        const int n = static_cast<int>(problem.c.size());

        Eigen::VectorXd x = Eigen::VectorXd::Constant(n, 1.0 / n);

        for (int it = 0; it < max_iters_; ++it) {
            // minimize c^T x, c = -signal  →  move towards +signal
            Eigen::VectorXd x_new = project_to_simplex(x + step_signal_ * (-problem.c));

            double diff = (x_new - x).norm();
            x = x_new;

            if (diff < tol_) {
                return {x, true, it + 1};
            }
        }

        double obj = -problem.c.dot(x);
        return {x, (obj > 0), max_iters_};
    }

    [[nodiscard]] const char* name() const override { return "LP (projected-gradient)"; }

private:
    double step_signal_;
    int max_iters_;
    double tol_;
};

} // namespace helix
