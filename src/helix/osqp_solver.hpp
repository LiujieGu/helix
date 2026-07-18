#pragma once

#include "solver.hpp"

#include <memory>

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

    // Discard factorization and warm-start state explicitly.
    void reset() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace helix
