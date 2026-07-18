#pragma once

#include <Eigen/Dense>

namespace helix {

struct QpResult {
    Eigen::VectorXd x;   // solution
    bool converged{false};
    int iterations{0};
};

// standard QP:  minimize  0.5 x^T H x + c^T x
//              subject to  A x = b
//                          x >= 0
struct QpProblem {
    Eigen::MatrixXd H;   // quadratic term (n x n, PSD)
    Eigen::VectorXd c;   // linear term (n)
    Eigen::MatrixXd A;   // equality constraints (p x n)
    Eigen::VectorXd b;   // equality RHS (p)
};

} // namespace helix
