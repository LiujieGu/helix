#pragma once

#include <Eigen/Sparse>

#include <limits>
#include <string>

namespace helix {

using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;

enum class SolveStatus {
    kSolved,
    kSolvedInaccurate,
    kPrimalInfeasible,
    kDualInfeasible,
    kMaxIterations,
    kTimeLimit,
    kNonConvex,
    kInterrupted,
    kInvalidProblem,
    kSetupFailure,
    kUnknown,
};

[[nodiscard]] inline const char* to_string(SolveStatus status) noexcept {
    switch (status) {
        case SolveStatus::kSolved:
            return "solved";
        case SolveStatus::kSolvedInaccurate:
            return "solved inaccurate";
        case SolveStatus::kPrimalInfeasible:
            return "primal infeasible";
        case SolveStatus::kDualInfeasible:
            return "dual infeasible";
        case SolveStatus::kMaxIterations:
            return "maximum iterations reached";
        case SolveStatus::kTimeLimit:
            return "time limit reached";
        case SolveStatus::kNonConvex:
            return "non-convex";
        case SolveStatus::kInterrupted:
            return "interrupted";
        case SolveStatus::kInvalidProblem:
            return "invalid problem";
        case SolveStatus::kSetupFailure:
            return "solver setup failed";
        case SolveStatus::kUnknown:
            return "unknown";
    }
    return "unknown";
}

struct SolverStatistics {
    int iterations{0};
    int rho_updates{0};
    double objective{std::numeric_limits<double>::quiet_NaN()};
    double primal_residual{std::numeric_limits<double>::quiet_NaN()};
    double dual_residual{std::numeric_limits<double>::quiet_NaN()};
    double setup_time_seconds{0.0};
    double solve_time_seconds{0.0};
    bool reused_workspace{false};
    bool updated_matrices{false};
};

struct SolveResult {
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    SolveStatus status{SolveStatus::kUnknown};
    SolverStatistics stats;
    std::string message;

    [[nodiscard]] bool success() const noexcept {
        return status == SolveStatus::kSolved || status == SolveStatus::kSolvedInaccurate;
    }
};

// Convex QP:
//     minimize    0.5 x' H x + c' x
//     subject to  lower <= A x <= upper
//
// H must be positive semidefinite. It may contain its upper triangular part only, or a full
// symmetric matrix. No constraints are added implicitly.
struct QpProblem {
    SparseMatrix H;
    Eigen::VectorXd c;
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

// LP in the same bound-constraint form. It is solved through the configured convex backend.
struct LpProblem {
    Eigen::VectorXd c;
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

struct SolverSettings {
    double absolute_tolerance{1e-6};
    double relative_tolerance{1e-6};
    int max_iterations{10'000};
    double time_limit_seconds{0.0};  // zero disables the limit
    bool warm_start{true};
    bool polish{true};
    bool verbose{false};
};

}  // namespace helix
