#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace helix {

// Euclidean projection onto {x | sum(x) = mass, x >= 0}. O(n log n).
inline Eigen::VectorXd project_to_simplex(const Eigen::Ref<const Eigen::VectorXd>& values,
                                          double mass = 1.0) {
    if (values.size() == 0) {
        throw std::invalid_argument("simplex projection requires a non-empty vector");
    }
    if (!values.allFinite() || !std::isfinite(mass) || mass <= 0.0) {
        throw std::invalid_argument("simplex projection requires finite values and positive mass");
    }

    std::vector<double> sorted(values.data(), values.data() + values.size());
    std::sort(sorted.begin(), sorted.end(), std::greater<>());

    double cumulative = 0.0;
    Eigen::Index active = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        cumulative += sorted[static_cast<std::size_t>(i)];
        const double threshold = (cumulative - mass) / static_cast<double>(i + 1);
        if (sorted[static_cast<std::size_t>(i)] > threshold) {
            active = i + 1;
        }
    }

    cumulative = 0.0;
    for (Eigen::Index i = 0; i < active; ++i) {
        cumulative += sorted[static_cast<std::size_t>(i)];
    }
    const double threshold = (cumulative - mass) / static_cast<double>(active);
    return (values.array() - threshold).max(0.0).matrix();
}

// Exact minimizer of c'x on a probability simplex. Ties are resolved by the first index.
inline Eigen::VectorXd minimize_linear_on_simplex(
    const Eigen::Ref<const Eigen::VectorXd>& objective, double mass = 1.0) {
    if (objective.size() == 0 || !objective.allFinite() || !std::isfinite(mass) || mass <= 0.0) {
        throw std::invalid_argument(
            "simplex LP requires a finite, non-empty objective and positive mass");
    }
    Eigen::Index best = 0;
    objective.minCoeff(&best);
    Eigen::VectorXd solution = Eigen::VectorXd::Zero(objective.size());
    solution[best] = mass;
    return solution;
}

}  // namespace helix
