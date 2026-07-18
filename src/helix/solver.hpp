#pragma once

#include "types.hpp"

namespace helix {

class QpSolver {
public:
    virtual ~QpSolver() = default;
    virtual SolveResult solve(const QpProblem& problem) = 0;
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

}  // namespace helix
