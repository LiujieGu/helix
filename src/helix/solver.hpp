#pragma once

#include "types.hpp"

namespace helix {

class Solver {
public:
    virtual ~Solver() = default;
    virtual QpResult solve(const QpProblem& problem) = 0;
    [[nodiscard]] virtual const char* name() const = 0;
};

} // namespace helix
