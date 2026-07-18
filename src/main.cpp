#include <iostream>
#include <vector>

#include "helix/lp_solver.hpp"
#include "helix/osqp_solver.hpp"
#include "helix/types.hpp"

int main() {
    const int n = 5;
    Eigen::VectorXd signal(n);
    signal << 0.1, 0.8, 0.3, 0.9, 0.5;

    // --- LP baseline: maximize signal^T x ---
    helix::QpProblem lp;
    lp.c = -signal;  // minimize -signal^T x
    lp.H = Eigen::MatrixXd::Zero(n, n);
    lp.A.resize(0, n);
    lp.b.resize(0);

    helix::LpSolver lp_solver;
    auto lp_res = lp_solver.solve(lp);
    std::cout << lp_solver.name() << ":\n"
              << "  weights: " << lp_res.x.transpose() << "\n"
              << "  signal= " << signal.dot(lp_res.x)
              << "  converged=" << lp_res.converged << "\n\n";

    // --- QP: add variance penalty (H = identity) ---
    helix::QpProblem qp;
    qp.H = Eigen::MatrixXd::Identity(n, n);
    qp.c = -signal;
    qp.A.resize(0, n);
    qp.b.resize(0);

    helix::OsqpSolver osqp;
    auto qp_res = osqp.solve(qp);
    std::cout << osqp.name() << ":\n"
              << "  weights: " << qp_res.x.transpose() << "\n"
              << "  signal= " << signal.dot(qp_res.x)
              << "  converged=" << qp_res.converged
              << "  iters=" << qp_res.iterations << "\n";

    return 0;
}
