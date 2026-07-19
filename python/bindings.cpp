#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "helix/lp_solver.hpp"
#include "helix/osqp_solver.hpp"
#include "helix/portfolio.hpp"

namespace py = pybind11;

namespace {

helix::SolveResult solve_qp(helix::OsqpSolver& solver, const helix::SparseMatrix& h,
                            const Eigen::VectorXd& c, const helix::SparseMatrix& a,
                            const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
    helix::QpProblem problem;
    problem.H = h;
    problem.c = c;
    problem.A = a;
    problem.lower = lower;
    problem.upper = upper;

    helix::SolveResult result;
    {
        py::gil_scoped_release release;
        result = solver.solve(problem);
    }
    return result;
}

helix::SolveResult solve_lp(helix::LpSolver& solver, const Eigen::VectorXd& c,
                            const helix::SparseMatrix& a, const Eigen::VectorXd& lower,
                            const Eigen::VectorXd& upper) {
    helix::LpProblem problem;
    problem.c = c;
    problem.A = a;
    problem.lower = lower;
    problem.upper = upper;

    helix::SolveResult result;
    {
        py::gil_scoped_release release;
        result = solver.solve(problem);
    }
    return result;
}

helix::PortfolioOptimizationInput make_portfolio_input(
    const Eigen::VectorXd& alpha, const Eigen::VectorXd& current_position_value,
    const Eigen::VectorXd& max_position_weight, const Eigen::VectorXd& liquidity_limit_value,
    double max_turnover_ratio, double capital) {
    helix::PortfolioOptimizationInput input;
    input.alpha = alpha;
    input.current_position_value = current_position_value;
    input.max_position_weight = max_position_weight;
    input.liquidity_limit_value = liquidity_limit_value;
    input.max_turnover_ratio = max_turnover_ratio;
    input.capital = capital;
    return input;
}

helix::PortfolioOptimizationResult solve_portfolio(helix::PortfolioLpOptimizer& optimizer,
                                                   helix::PortfolioOptimizationInput input) {
    helix::PortfolioOptimizationResult result;
    {
        py::gil_scoped_release release;
        result = optimizer.solve(input);
    }
    return result;
}

}  // namespace

PYBIND11_MODULE(helix, module) {
    module.doc() = "Python bindings for the Helix convex optimization library";
    module.attr("__version__") = HELIX_VERSION;

    py::enum_<helix::SolveStatus>(module, "SolveStatus")
        .value("SOLVED", helix::SolveStatus::kSolved)
        .value("SOLVED_INACCURATE", helix::SolveStatus::kSolvedInaccurate)
        .value("PRIMAL_INFEASIBLE", helix::SolveStatus::kPrimalInfeasible)
        .value("DUAL_INFEASIBLE", helix::SolveStatus::kDualInfeasible)
        .value("MAX_ITERATIONS", helix::SolveStatus::kMaxIterations)
        .value("TIME_LIMIT", helix::SolveStatus::kTimeLimit)
        .value("NON_CONVEX", helix::SolveStatus::kNonConvex)
        .value("INTERRUPTED", helix::SolveStatus::kInterrupted)
        .value("INVALID_PROBLEM", helix::SolveStatus::kInvalidProblem)
        .value("SETUP_FAILURE", helix::SolveStatus::kSetupFailure)
        .value("UNKNOWN", helix::SolveStatus::kUnknown);

    py::class_<helix::SolverSettings>(module, "SolverSettings")
        .def(py::init<>())
        .def_readwrite("absolute_tolerance", &helix::SolverSettings::absolute_tolerance)
        .def_readwrite("relative_tolerance", &helix::SolverSettings::relative_tolerance)
        .def_readwrite("max_iterations", &helix::SolverSettings::max_iterations)
        .def_readwrite("time_limit_seconds", &helix::SolverSettings::time_limit_seconds)
        .def_readwrite("warm_start", &helix::SolverSettings::warm_start)
        .def_readwrite("polish", &helix::SolverSettings::polish)
        .def_readwrite("verbose", &helix::SolverSettings::verbose);

    py::class_<helix::SolverStatistics>(module, "SolverStatistics")
        .def_property_readonly(
            "iterations", [](const helix::SolverStatistics& stats) { return stats.iterations; })
        .def_property_readonly(
            "rho_updates", [](const helix::SolverStatistics& stats) { return stats.rho_updates; })
        .def_property_readonly("objective",
                               [](const helix::SolverStatistics& stats) { return stats.objective; })
        .def_property_readonly(
            "primal_residual",
            [](const helix::SolverStatistics& stats) { return stats.primal_residual; })
        .def_property_readonly(
            "dual_residual",
            [](const helix::SolverStatistics& stats) { return stats.dual_residual; })
        .def_property_readonly(
            "setup_time_seconds",
            [](const helix::SolverStatistics& stats) { return stats.setup_time_seconds; })
        .def_property_readonly(
            "solve_time_seconds",
            [](const helix::SolverStatistics& stats) { return stats.solve_time_seconds; })
        .def_property_readonly(
            "reused_workspace",
            [](const helix::SolverStatistics& stats) { return stats.reused_workspace; })
        .def_property_readonly("updated_matrices", [](const helix::SolverStatistics& stats) {
            return stats.updated_matrices;
        });

    py::class_<helix::SolveResult>(module, "SolveResult")
        .def_property_readonly("x", [](const helix::SolveResult& result) { return result.x; })
        .def_property_readonly("y", [](const helix::SolveResult& result) { return result.y; })
        .def_property_readonly("status",
                               [](const helix::SolveResult& result) { return result.status; })
        .def_property_readonly(
            "status_name",
            [](const helix::SolveResult& result) { return helix::to_string(result.status); })
        .def_property_readonly(
            "objective", [](const helix::SolveResult& result) { return result.stats.objective; })
        .def_property_readonly(
            "stats",
            [](helix::SolveResult& result) -> helix::SolverStatistics& { return result.stats; },
            py::return_value_policy::reference_internal)
        .def_property_readonly("message",
                               [](const helix::SolveResult& result) { return result.message; })
        .def_property_readonly("success",
                               [](const helix::SolveResult& result) { return result.success(); });

    py::class_<helix::OsqpSolver>(module, "OsqpSolver")
        .def(py::init<helix::SolverSettings>(), py::arg("settings") = helix::SolverSettings{})
        .def("solve", &solve_qp, py::arg("H"), py::arg("c"), py::arg("A"), py::arg("lower"),
             py::arg("upper"), "Solve a convex QP using SciPy sparse matrices and NumPy vectors.")
        .def("reset", &helix::OsqpSolver::reset)
        .def_property_readonly("name", &helix::OsqpSolver::name);

    py::class_<helix::LpSolver>(module, "LpSolver")
        .def(py::init<helix::SolverSettings>(), py::arg("settings") = helix::SolverSettings{})
        .def("solve", &solve_lp, py::arg("c"), py::arg("A"), py::arg("lower"), py::arg("upper"),
             "Solve an LP using a SciPy sparse matrix and NumPy vectors.")
        .def("reset", &helix::LpSolver::reset)
        .def_property_readonly("name", &helix::LpSolver::name);

    py::class_<helix::PortfolioOptimizationInput>(module, "PortfolioOptimizationInput")
        .def(py::init<>())
        .def_readwrite("alpha", &helix::PortfolioOptimizationInput::alpha)
        .def_readwrite("current_position_value",
                       &helix::PortfolioOptimizationInput::current_position_value)
        .def_readwrite("max_position_weight",
                       &helix::PortfolioOptimizationInput::max_position_weight)
        .def_readwrite("liquidity_limit_value",
                       &helix::PortfolioOptimizationInput::liquidity_limit_value)
        .def_readwrite("max_turnover_ratio", &helix::PortfolioOptimizationInput::max_turnover_ratio)
        .def_readwrite("capital", &helix::PortfolioOptimizationInput::capital);

    py::class_<helix::PortfolioOptimizationResult>(module, "PortfolioOptimizationResult")
        .def_property_readonly("target_position_value",
                               [](const helix::PortfolioOptimizationResult& result) {
                                   return result.target_position_value;
                               })
        .def_property_readonly(
            "target_weight",
            [](const helix::PortfolioOptimizationResult& result) { return result.target_weight; })
        .def_property_readonly(
            "trade_value",
            [](const helix::PortfolioOptimizationResult& result) { return result.trade_value; })
        .def_property_readonly(
            "expected_alpha",
            [](const helix::PortfolioOptimizationResult& result) { return result.expected_alpha; })
        .def_property_readonly(
            "turnover_ratio",
            [](const helix::PortfolioOptimizationResult& result) { return result.turnover_ratio; })
        .def_property_readonly(
            "solver_result",
            [](helix::PortfolioOptimizationResult& result) -> helix::SolveResult& {
                return result.solver_result;
            },
            py::return_value_policy::reference_internal)
        .def_property_readonly("success", [](const helix::PortfolioOptimizationResult& result) {
            return result.success();
        });

    py::class_<helix::PortfolioLpOptimizer>(module, "PortfolioLpOptimizer")
        .def(py::init<helix::SolverSettings>(), py::arg("settings") = helix::SolverSettings{})
        .def("solve", &solve_portfolio, py::arg("input"))
        .def(
            "solve",
            [](helix::PortfolioLpOptimizer& optimizer, const Eigen::VectorXd& alpha,
               const Eigen::VectorXd& current_position_value,
               const Eigen::VectorXd& max_position_weight,
               const Eigen::VectorXd& liquidity_limit_value, double max_turnover_ratio,
               double capital) {
                return solve_portfolio(
                    optimizer,
                    make_portfolio_input(alpha, current_position_value, max_position_weight,
                                         liquidity_limit_value, max_turnover_ratio, capital));
            },
            py::arg("alpha"), py::arg("current_position_value"), py::arg("max_position_weight"),
            py::arg("liquidity_limit_value"), py::arg("max_turnover_ratio"), py::arg("capital"))
        .def("reset", &helix::PortfolioLpOptimizer::reset);

    module.def(
        "optimize_portfolio_lp",
        [](const Eigen::VectorXd& alpha, const Eigen::VectorXd& current_position_value,
           const Eigen::VectorXd& max_position_weight, const Eigen::VectorXd& liquidity_limit_value,
           double max_turnover_ratio, double capital, helix::SolverSettings settings) {
            helix::PortfolioLpOptimizer optimizer(settings);
            return solve_portfolio(
                optimizer,
                make_portfolio_input(alpha, current_position_value, max_position_weight,
                                     liquidity_limit_value, max_turnover_ratio, capital));
        },
        py::arg("alpha"), py::arg("current_position_value"), py::arg("max_position_weight"),
        py::arg("liquidity_limit_value"), py::arg("max_turnover_ratio"), py::arg("capital"),
        py::arg("settings") = helix::SolverSettings{},
        "One-off long-only portfolio LP optimization.");
}
