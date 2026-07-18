#include "helix/simplex.hpp"

#include "test_utils.hpp"

#include <Eigen/Dense>

#include <stdexcept>

int main() {
    Eigen::VectorXd values(4);
    values << -0.2, 0.1, 1.4, 0.5;
    const Eigen::VectorXd projected = helix::project_to_simplex(values);
    TEST_REQUIRE(near(projected.sum(), 1.0, 1e-12));
    TEST_REQUIRE(projected.minCoeff() >= 0.0);

    Eigen::VectorXd objective(4);
    objective << 3.0, -2.0, 1.0, 0.0;
    const Eigen::VectorXd exact = helix::minimize_linear_on_simplex(objective);
    TEST_REQUIRE(exact[1] == 1.0);
    TEST_REQUIRE(exact.sum() == 1.0);

    bool rejected_empty = false;
    try {
        helix::project_to_simplex(Eigen::VectorXd{});
    } catch (const std::invalid_argument&) {
        rejected_empty = true;
    }
    TEST_REQUIRE(rejected_empty);
    return 0;
}
