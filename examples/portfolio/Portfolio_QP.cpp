#include "helix/osqp_solver.hpp"

#include "portfolio_common.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

void validate_covariance(const Eigen::MatrixXd& covariance, Eigen::Index assets) {
    if (covariance.rows() != assets || covariance.cols() != assets ||
        !covariance.allFinite()) {
        throw std::invalid_argument("covariance must be a finite n by n matrix");
    }
    if (!covariance.isApprox(covariance.transpose(), 1e-12)) {
        throw std::invalid_argument("covariance must be symmetric");
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenvalues(covariance, Eigen::EigenvaluesOnly);
    if (eigenvalues.info() != Eigen::Success || eigenvalues.eigenvalues().minCoeff() < -1e-12) {
        throw std::invalid_argument("covariance must be positive semidefinite");
    }
}

helix::QpProblem make_portfolio_qp(const helix_examples::PortfolioInput& input,
                                   const Eigen::MatrixXd& covariance,
                                   double risk_aversion) {
    helix_examples::validate_input(input);
    validate_covariance(covariance, input.alpha.size());
    if (!std::isfinite(risk_aversion) || risk_aversion <= 0.0) {
        throw std::invalid_argument("risk_aversion must be finite and positive");
    }

    const Eigen::Index assets = input.alpha.size();
    const Eigen::Index variables = 2 * assets;
    const helix_examples::PortfolioConstraints constraints =
        helix_examples::make_portfolio_constraints(input);

    // Mean-variance objective:
    //     maximize alpha'w - 0.5 * risk_aversion * w'covariance*w.
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
    hessian.topLeftCorner(assets, assets) = risk_aversion * covariance;

    helix::QpProblem problem;
    problem.H = hessian.sparseView();
    problem.c = Eigen::VectorXd::Zero(variables);
    problem.c.head(assets) = -input.alpha;
    problem.A = constraints.matrix;
    problem.lower = constraints.lower;
    problem.upper = constraints.upper;
    return problem;
}

}  // namespace

int main() {
    const helix_examples::PortfolioInput input = helix_examples::make_example_input();

    // Annualized covariance in the same asset order as alpha.
    Eigen::MatrixXd covariance(4, 4);
    covariance << 0.040, 0.006, 0.008, 0.004, 0.006, 0.090, 0.010, 0.012, 0.008,
        0.010, 0.160, 0.014, 0.004, 0.012, 0.014, 0.250;
    constexpr double risk_aversion = 2.0;
    constexpr double risk_free_rate = 0.0;

    helix::SolverSettings settings;
    settings.absolute_tolerance = 1e-7;
    settings.relative_tolerance = 1e-7;
    settings.polish = true;
    helix::OsqpSolver solver(settings);

    try {
        const helix::QpProblem problem = make_portfolio_qp(input, covariance, risk_aversion);
        helix_examples::PortfolioResult result =
            helix_examples::decode_result(input, solver.solve(problem));
        if (!result.solve_result.success()) {
            std::cerr << "portfolio QP failed: " << result.solve_result.message << '\n';
            return 1;
        }

        helix_examples::print_result(input, result);
        const double variance = result.target_weight.dot(covariance * result.target_weight);
        const double volatility = std::sqrt(std::max(0.0, variance));
        const double sharpe = volatility > 0.0
                                  ? (result.expected_alpha - risk_free_rate) / volatility
                                  : 0.0;
        std::cout << "risk aversion: " << risk_aversion << '\n'
                  << "volatility: " << volatility << '\n'
                  << "Sharpe (risk-free=" << risk_free_rate << "): " << sharpe << '\n';
    } catch (const std::exception& error) {
        std::cerr << "invalid portfolio input: " << error.what() << '\n';
        return 2;
    }
    return 0;
}
