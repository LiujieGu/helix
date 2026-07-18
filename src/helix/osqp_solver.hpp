#pragma once

#include "solver.hpp"

#include <memory>

namespace helix {

class OsqpSolver : public Solver {
public:
    OsqpSolver();
    ~OsqpSolver() override;

    QpResult solve(const QpProblem& problem) override;
    [[nodiscard]] const char* name() const override { return "OSQP"; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace helix
