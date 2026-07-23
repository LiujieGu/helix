#include "osqp_solver.hpp"

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helix {
namespace {

struct CscStorage {
    std::vector<OSQPInt> column_pointers;
    std::vector<OSQPInt> row_indices;
    std::vector<OSQPFloat> values;

    void assign(const SparseMatrix& matrix) {
        column_pointers.resize(static_cast<std::size_t>(matrix.cols() + 1));
        row_indices.resize(static_cast<std::size_t>(matrix.nonZeros()));
        values.resize(static_cast<std::size_t>(matrix.nonZeros()));

        for (Eigen::Index column = 0; column <= matrix.cols(); ++column) {
            column_pointers[static_cast<std::size_t>(column)] =
                static_cast<OSQPInt>(matrix.outerIndexPtr()[column]);
        }
        for (Eigen::Index index = 0; index < matrix.nonZeros(); ++index) {
            row_indices[static_cast<std::size_t>(index)] =
                static_cast<OSQPInt>(matrix.innerIndexPtr()[index]);
            values[static_cast<std::size_t>(index)] =
                static_cast<OSQPFloat>(matrix.valuePtr()[index]);
        }
    }

    [[nodiscard]] OSQPCscMatrix view(OSQPInt rows, OSQPInt columns) {
        return OSQPCscMatrix{rows,
                             columns,
                             column_pointers.data(),
                             row_indices.empty() ? nullptr : row_indices.data(),
                             values.empty() ? nullptr : values.data(),
                             static_cast<OSQPInt>(values.size()),
                             -1,
                             0};
    }

    [[nodiscard]] bool same_pattern(const CscStorage& other) const noexcept {
        return column_pointers == other.column_pointers && row_indices == other.row_indices;
    }
};

bool sparse_values_are_finite(const SparseMatrix& matrix) {
    for (Eigen::Index index = 0; index < matrix.nonZeros(); ++index) {
        if (!std::isfinite(matrix.valuePtr()[index])) {
            return false;
        }
    }
    return true;
}

std::string validate_problem(const QpProblem& problem) {
    const Eigen::Index variables = problem.c.size();
    if (variables <= 0) {
        return "c must contain at least one variable";
    }
    if (problem.H.rows() != variables || problem.H.cols() != variables) {
        return "H must be square and match c.size()";
    }
    if (problem.A.cols() != variables) {
        return "A.cols() must match c.size()";
    }
    if (problem.lower.size() != problem.A.rows() || problem.upper.size() != problem.A.rows()) {
        return "lower and upper must match A.rows()";
    }
    if (!problem.c.allFinite() || !sparse_values_are_finite(problem.H) ||
        !sparse_values_are_finite(problem.A)) {
        return "objective and matrix coefficients must be finite";
    }

    for (Eigen::Index row = 0; row < problem.A.rows(); ++row) {
        if (std::isnan(problem.lower[row]) || std::isnan(problem.upper[row]) ||
            problem.lower[row] > problem.upper[row]) {
            return "every constraint must satisfy lower <= upper and contain no NaN";
        }
        if (problem.lower[row] == std::numeric_limits<double>::infinity() ||
            problem.upper[row] == -std::numeric_limits<double>::infinity()) {
            return "a lower bound cannot be +infinity and an upper bound cannot be -infinity";
        }
    }

    // OSQP consumes only the upper triangle. If a lower triangle is supplied, require the
    // matching upper coefficient so asymmetric input is never silently discarded.
    constexpr double symmetry_tolerance = 1e-12;
    for (int column = 0; column < problem.H.outerSize(); ++column) {
        for (SparseMatrix::InnerIterator entry(problem.H, column); entry; ++entry) {
            if (entry.row() <= entry.col()) {
                continue;
            }
            const double mirrored = problem.H.coeff(entry.col(), entry.row());
            const double scale = 1.0 + std::max(std::abs(entry.value()), std::abs(mirrored));
            if (std::abs(entry.value() - mirrored) > symmetry_tolerance * scale) {
                return "H must be symmetric, or contain only its upper triangular part";
            }
        }
    }
    return {};
}

SparseMatrix upper_triangle(const SparseMatrix& matrix) {
    SparseMatrix upper = matrix.template triangularView<Eigen::Upper>();
    upper.makeCompressed();
    return upper;
}

SparseMatrix compressed_copy(const SparseMatrix& matrix) {
    SparseMatrix result = matrix;
    result.makeCompressed();
    return result;
}

std::vector<OSQPFloat> copy_vector(const Eigen::VectorXd& vector, bool bounds = false) {
    std::vector<OSQPFloat> result(static_cast<std::size_t>(vector.size()));
    for (Eigen::Index index = 0; index < vector.size(); ++index) {
        double value = vector[index];
        if (bounds && std::isinf(value)) {
            value = std::signbit(value) ? -OSQP_INFTY : OSQP_INFTY;
        }
        result[static_cast<std::size_t>(index)] = static_cast<OSQPFloat>(value);
    }
    return result;
}

SolveStatus map_status(OSQPInt status) noexcept {
    switch (status) {
        case OSQP_SOLVED:
            return SolveStatus::kSolved;
        case OSQP_SOLVED_INACCURATE:
            return SolveStatus::kSolvedInaccurate;
        case OSQP_PRIMAL_INFEASIBLE:
        case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
            return SolveStatus::kPrimalInfeasible;
        case OSQP_DUAL_INFEASIBLE:
        case OSQP_DUAL_INFEASIBLE_INACCURATE:
            return SolveStatus::kDualInfeasible;
        case OSQP_MAX_ITER_REACHED:
            return SolveStatus::kMaxIterations;
        case OSQP_TIME_LIMIT_REACHED:
            return SolveStatus::kTimeLimit;
        case OSQP_NON_CVX:
            return SolveStatus::kNonConvex;
        case OSQP_SIGINT:
            return SolveStatus::kInterrupted;
        default:
            return SolveStatus::kUnknown;
    }
}

}  // namespace

struct OsqpSolver::Impl {
    explicit Impl(SolverSettings requested_settings) : settings(std::move(requested_settings)) {
        osqp_set_default_settings(&osqp_settings);
        osqp_settings.eps_abs = settings.absolute_tolerance;
        osqp_settings.eps_rel = settings.relative_tolerance;
        osqp_settings.max_iter = settings.max_iterations;
        osqp_settings.time_limit = settings.time_limit_seconds > 0.0
                                       ? settings.time_limit_seconds
                                       : static_cast<OSQPFloat>(OSQP_TIME_LIMIT);
        osqp_settings.warm_starting = settings.warm_start ? 1 : 0;
        osqp_settings.polishing = settings.polish ? 1 : 0;
        osqp_settings.verbose = settings.verbose ? 1 : 0;
    }

    ~Impl() { reset(); }

    void reset() noexcept {
        if (solver != nullptr) {
            osqp_cleanup(solver);
            solver = nullptr;
        }
        p_storage = {};
        a_storage = {};
        rows = 0;
        columns = 0;
    }

    SolverSettings settings;
    OSQPSettings osqp_settings{};
    OSQPSolver* solver{nullptr};
    CscStorage p_storage;
    CscStorage a_storage;
    OSQPInt rows{0};
    OSQPInt columns{0};
};

OsqpSolver::OsqpSolver(SolverSettings settings) : impl_(std::make_unique<Impl>(settings)) {}
OsqpSolver::~OsqpSolver() = default;
OsqpSolver::OsqpSolver(OsqpSolver&&) noexcept = default;
OsqpSolver& OsqpSolver::operator=(OsqpSolver&&) noexcept = default;

void OsqpSolver::reset() noexcept {
    if (impl_) {
        impl_->reset();
    }
}

bool OsqpSolver::has_workspace() const noexcept {
    return impl_ && impl_->solver != nullptr;
}

void OsqpSolver::warm_start(const Eigen::VectorXd& primal, const Eigen::VectorXd& dual) {
    if (!has_workspace()) {
        throw std::logic_error("solve() must create a workspace before warm_start()");
    }
    auto& state = *impl_;
    if (!state.settings.warm_start) {
        throw std::logic_error("warm_start is disabled in SolverSettings");
    }
    if (primal.size() == 0 && dual.size() == 0) {
        throw std::invalid_argument("at least one of primal or dual must be provided");
    }
    if (primal.size() != 0 && primal.size() != state.columns) {
        throw std::invalid_argument("primal must match the number of variables");
    }
    if (dual.size() != 0 && dual.size() != state.rows) {
        throw std::invalid_argument("dual must match the number of constraints");
    }
    if (!primal.allFinite() || !dual.allFinite()) {
        throw std::invalid_argument("warm-start vectors must be finite");
    }

    const auto primal_values = copy_vector(primal);
    const auto dual_values = copy_vector(dual);
    const OSQPInt error =
        osqp_warm_start(state.solver, primal.size() == 0 ? nullptr : primal_values.data(),
                        dual.size() == 0 ? nullptr : dual_values.data());
    if (error != 0) {
        throw std::runtime_error("OSQP rejected warm start with error code " +
                                 std::to_string(error));
    }
}

SolveResult OsqpSolver::solve(const QpProblem& problem) {
    SolveResult result;
    const std::string validation_error = validate_problem(problem);
    if (!validation_error.empty()) {
        result.status = SolveStatus::kInvalidProblem;
        result.message = validation_error;
        return result;
    }

    SparseMatrix p_matrix = upper_triangle(problem.H);
    SparseMatrix a_matrix = compressed_copy(problem.A);
    CscStorage new_p;
    CscStorage new_a;
    new_p.assign(p_matrix);
    new_a.assign(a_matrix);

    const OSQPInt variables = static_cast<OSQPInt>(problem.c.size());
    const OSQPInt constraints = static_cast<OSQPInt>(problem.A.rows());
    auto q = copy_vector(problem.c);
    auto lower = copy_vector(problem.lower, true);
    auto upper = copy_vector(problem.upper, true);

    auto& state = *impl_;
    const bool can_reuse = state.solver != nullptr && state.columns == variables &&
                           state.rows == constraints && state.p_storage.same_pattern(new_p) &&
                           state.a_storage.same_pattern(new_a);

    if (can_reuse) {
        const bool matrices_changed =
            state.p_storage.values != new_p.values || state.a_storage.values != new_a.values;
        OSQPInt matrix_error = 0;
        if (matrices_changed) {
            matrix_error = osqp_update_data_mat(
                state.solver, new_p.values.empty() ? nullptr : new_p.values.data(), nullptr,
                static_cast<OSQPInt>(new_p.values.size()),
                new_a.values.empty() ? nullptr : new_a.values.data(), nullptr,
                static_cast<OSQPInt>(new_a.values.size()));
        }
        const OSQPInt vector_error =
            osqp_update_data_vec(state.solver, q.data(), lower.data(), upper.data());
        if (matrix_error != 0 || vector_error != 0) {
            state.reset();
            result.status = SolveStatus::kSetupFailure;
            result.message = "OSQP rejected a workspace data update";
            return result;
        }
        state.p_storage = std::move(new_p);
        state.a_storage = std::move(new_a);
        result.stats.reused_workspace = true;
        result.stats.updated_matrices = matrices_changed;
    } else {
        state.reset();
        state.p_storage = std::move(new_p);
        state.a_storage = std::move(new_a);
        state.rows = constraints;
        state.columns = variables;

        OSQPCscMatrix p_view = state.p_storage.view(variables, variables);
        OSQPCscMatrix a_view = state.a_storage.view(constraints, variables);
        const OSQPInt setup_error =
            osqp_setup(&state.solver, &p_view, q.data(), &a_view, lower.data(), upper.data(),
                       constraints, variables, &state.osqp_settings);
        if (setup_error != 0) {
            state.reset();
            result.status = SolveStatus::kSetupFailure;
            result.message = "OSQP setup failed with error code " + std::to_string(setup_error);
            return result;
        }
    }

    if (!state.settings.warm_start) {
        osqp_cold_start(state.solver);
    }
    const OSQPInt solve_error = osqp_solve(state.solver);
    if (solve_error != 0 || state.solver->info == nullptr) {
        result.status = SolveStatus::kUnknown;
        result.message = "OSQP solve failed with error code " + std::to_string(solve_error);
        return result;
    }

    const OSQPInfo& info = *state.solver->info;
    result.status = map_status(info.status_val);
    result.message = info.status;
    result.stats.iterations = static_cast<int>(info.iter);
    result.stats.rho_updates = static_cast<int>(info.rho_updates);
    result.stats.objective = info.obj_val;
    result.stats.primal_residual = info.prim_res;
    result.stats.dual_residual = info.dual_res;
    result.stats.setup_time_seconds = info.setup_time;
    result.stats.solve_time_seconds = info.solve_time;

    if (result.success() && state.solver->solution != nullptr) {
        result.x.resize(problem.c.size());
        result.y.resize(problem.A.rows());
        for (Eigen::Index index = 0; index < result.x.size(); ++index) {
            result.x[index] = state.solver->solution->x[index];
        }
        for (Eigen::Index index = 0; index < result.y.size(); ++index) {
            result.y[index] = state.solver->solution->y[index];
        }
    }
    return result;
}

}  // namespace helix
