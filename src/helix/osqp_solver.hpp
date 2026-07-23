#pragma once

#include <memory>

#include "solver.hpp"

namespace helix {

class OsqpSolver final : public QpSolver {
  public:
    explicit OsqpSolver(SolverSettings settings = {});
    ~OsqpSolver() override;

    OsqpSolver(OsqpSolver&&) noexcept;
    OsqpSolver& operator=(OsqpSolver&&) noexcept;
    OsqpSolver(const OsqpSolver&) = delete;
    OsqpSolver& operator=(const OsqpSolver&) = delete;

    SolveResult solve(const QpProblem& problem) override;
    [[nodiscard]] const char* name() const noexcept override { return "OSQP"; }

    // Override the retained OSQP primal/dual iterate before the next solve. Empty vectors leave
    // that side unchanged. A workspace must already have been created by solve().
    void warm_start(const Eigen::VectorXd& primal = {}, const Eigen::VectorXd& dual = {});
    [[nodiscard]] bool has_workspace() const noexcept;

    // Discard factorization and warm-start state explicitly.
    void reset() noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace helix
