#pragma once

#include <Eigen/Dense>

namespace helix {

// Project v onto probability simplex {x : sum(x) = 1, x_i >= 0} in O(n log n).
// After sorting v_desc, find idx where threshold satisfies the constraint exactly,
// then clamp to zero and shift for sum=1.
inline Eigen::VectorXd project_to_simplex(const Eigen::VectorXd& v) {
    const int n = static_cast<int>(v.size());
    Eigen::VectorXd y = v;

    struct Pair { double val; int idx; };
    std::vector<Pair> pairs(n);
    for (int i = 0; i < n; ++i) pairs[i] = {v[i], i};
    std::sort(pairs.begin(), pairs.end(), [](auto a, auto b) { return a.val > b.val; });

    double cum = 0.0;
    int k = 0;
    for (int i = 0; i < n; ++i) {
        cum += pairs[i].val;
        double thresh = (cum - 1.0) / (i + 1);
        if (pairs[i].val > thresh) k = i + 1;
    }

    double tau = 0.0;
    for (int i = 0; i < k; ++i) tau += pairs[i].val;
    tau = (tau - 1.0) / k;

    y.setZero();
    for (int i = 0; i < k; ++i) y[pairs[i].idx] = pairs[i].val - tau;
    return y;
}

} // namespace helix
