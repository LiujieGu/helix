#include "helix/lp_solver.hpp"
#include "helix/types.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    const int n = 5;
    Eigen::VectorXd signal(n);
    signal << 0.1, 0.8, 0.3, 0.9, 0.5;

    helix::QpProblem lp;
    lp.c = -signal;
    lp.H = Eigen::MatrixXd::Zero(n, n);
    lp.A.resize(0, n);
    lp.b.resize(0);

    helix::LpSolver solver;
    auto res = solver.solve(lp);

    assert(res.converged);

    // sum(x) == 1
    assert(std::abs(res.x.sum() - 1.0) < 1e-6);

    // x >= 0
    for (int i = 0; i < n; ++i)
        assert(res.x[i] >= -1e-10);

    // optimal: weight on max-signal index (3)
    assert(res.x[3] > 0.5);

    std::cout << "LP test passed\n";
    return 0;
}
