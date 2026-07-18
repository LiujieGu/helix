#include "helix/osqp_solver.hpp"
#include "helix/types.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    const int n = 5;
    Eigen::VectorXd signal(n);
    signal << 0.1, 0.8, 0.3, 0.9, 0.5;

    helix::QpProblem qp;
    qp.H = Eigen::MatrixXd::Identity(n, n);
    qp.c = -signal;
    qp.A.resize(0, n);
    qp.b.resize(0);

    helix::OsqpSolver solver;
    auto res = solver.solve(qp);

    assert(res.converged);

    // sum(x) == 1
    assert(std::abs(res.x.sum() - 1.0) < 1e-4);

    // x >= 0
    for (int i = 0; i < n; ++i)
        assert(res.x[i] >= -1e-8);

    std::cout << "QP test passed\n";
    return 0;
}
