#include "osqp_solver.hpp"

#include <osqp.h>   // pulls in all OSQP public headers

#include <Eigen/Sparse>
#include <cstdlib>
#include <memory>
#include <vector>

namespace helix {

namespace {

using SparseMat = Eigen::SparseMatrix<double, Eigen::ColMajor>;

void set_sparse_data(OSQPCscMatrix* M, const SparseMat& mat) {
    OSQPInt nzmax = static_cast<OSQPInt>(mat.nonZeros());
    if (!M->p || M->nzmax < nzmax) {
        if (M->p) { free(M->p); free(M->i); free(M->x); }
        M->p = static_cast<OSQPInt*>(malloc((mat.cols() + 1) * sizeof(OSQPInt)));
        M->i = static_cast<OSQPInt*>(malloc(nzmax * sizeof(OSQPInt)));
        M->x = static_cast<OSQPFloat*>(malloc(nzmax * sizeof(OSQPFloat)));
        M->nzmax = nzmax;
    }
    M->m = static_cast<OSQPInt>(mat.rows());
    M->n = static_cast<OSQPInt>(mat.cols());
    M->nz = nzmax;

    const int* outer = mat.outerIndexPtr();
    const int* inner = mat.innerIndexPtr();
    const double* values = mat.valuePtr();

    M->p[0] = 0;
    for (int j = 0; j < mat.outerSize(); ++j)
        M->p[j + 1] = static_cast<OSQPInt>(outer[j + 1]);
    for (int k = 0; k < mat.nonZeros(); ++k) {
        M->i[k] = static_cast<OSQPInt>(inner[k]);
        M->x[k] = static_cast<OSQPFloat>(values[k]);
    }
}

void clear_csc(OSQPCscMatrix* M) {
    if (!M) return;
    if (M->p) { free(M->p); M->p = nullptr; }
    if (M->i) { free(M->i); M->i = nullptr; }
    if (M->x) { free(M->x); M->x = nullptr; }
    M->nzmax = 0;
}

} // namespace

struct OsqpSolver::Impl {
    OSQPSolver* solver{nullptr};
    OSQPSettings* settings{nullptr};
    SparseMat A_sparse;
    SparseMat H_sparse;
    Eigen::VectorXd l, u;

    Impl() {
        settings = reinterpret_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
        osqp_set_default_settings(settings);
        settings->warm_starting = 1;
        settings->verbose = 0;
    }

    ~Impl() {
        if (solver) osqp_cleanup(solver);
        free(settings);
    }
};

OsqpSolver::OsqpSolver() : impl_(std::make_unique<Impl>()) {}
OsqpSolver::~OsqpSolver() = default;

QpResult OsqpSolver::solve(const QpProblem& problem) {
    auto& p = *impl_;
    const OSQPInt n = static_cast<OSQPInt>(problem.H.rows());
    const OSQPInt m_eq = static_cast<OSQPInt>(problem.A.rows());
    const OSQPInt m = m_eq + 1 + n; // +1 for sum(x)=1, +n for x >= 0

    p.H_sparse = problem.H.sparseView();

    // A = [original equality constraints; ones_row (sum=1); I_n (x >= 0)]
    {
        std::vector<Eigen::Triplet<double>> triplets;
        SparseMat A_problem = problem.A.sparseView();
        for (int k = 0; k < A_problem.outerSize(); ++k)
            for (SparseMat::InnerIterator it(A_problem, k); it; ++it)
                triplets.emplace_back(it.row(), it.col(), it.value());
        // sum(x) = 1 row
        for (OSQPInt j = 0; j < n; ++j)
            triplets.emplace_back(m_eq, j, 1.0);
        // x_i >= 0 rows (identity block)
        for (OSQPInt j = 0; j < n; ++j)
            triplets.emplace_back(m_eq + 1 + j, j, 1.0);
        p.A_sparse.resize(m, n);
        p.A_sparse.setFromTriplets(triplets.begin(), triplets.end());
    }

    p.l.resize(m); p.u.resize(m);
    for (int i = 0; i < m_eq; ++i) {
        p.l[i] = problem.b[i];
        p.u[i] = problem.b[i];
    }
    p.l[m_eq] = 1.0;  p.u[m_eq] = 1.0;   // sum(x) = 1
    for (OSQPInt j = 0; j < n; ++j) {
        p.l[m_eq + 1 + j] = 0.0;           // x_j >= 0
        p.u[m_eq + 1 + j] = OSQP_INFTY;
    }

    OSQPCscMatrix P_mat{};
    OSQPCscMatrix A_mat{};
    set_sparse_data(&P_mat, p.H_sparse);
    set_sparse_data(&A_mat, p.A_sparse);

    std::vector<OSQPFloat> q_vec(n), l_vec(m), u_vec(m);
    for (int i = 0; i < n; ++i) q_vec[i] = static_cast<OSQPFloat>(problem.c[i]);
    for (int i = 0; i < m; ++i) l_vec[i] = static_cast<OSQPFloat>(p.l[i]);
    for (int i = 0; i < m; ++i) u_vec[i] = static_cast<OSQPFloat>(p.u[i]);

    if (p.solver) { osqp_cleanup(p.solver); p.solver = nullptr; }
    OSQPInt exitflag = osqp_setup(&p.solver,
                                  &P_mat, q_vec.data(),
                                  &A_mat, l_vec.data(), u_vec.data(),
                                  m, n, p.settings);
    if (exitflag != 0) {
        clear_csc(&P_mat); clear_csc(&A_mat);
        return {Eigen::VectorXd::Zero(static_cast<int>(n)), false, 0};
    }

    osqp_warm_start(p.solver, nullptr, nullptr);
    osqp_solve(p.solver);

    QpResult result;
    if (p.solver->info->status_val == OSQP_SOLVED ||
        p.solver->info->status_val == OSQP_SOLVED_INACCURATE) {
        result.converged = true;
        result.x.resize(static_cast<int>(n));
        for (int i = 0; i < n; ++i) result.x[i] = p.solver->solution->x[i];
        result.iterations = static_cast<int>(p.solver->info->iter);
    } else {
        result.converged = false;
        result.x = Eigen::VectorXd::Zero(static_cast<int>(n));
    }

    clear_csc(&P_mat); clear_csc(&A_mat);
    return result;
}

} // namespace helix
