import numpy as np

import helix


def portfolio_data():
    return {
        "alpha": np.array([0.02, 0.08, 0.05, -0.01]),
        "current_position_value": np.array([300_000.0, 250_000.0, 250_000.0, 200_000.0]),
        "max_position_weight": np.array([0.35, 0.40, 0.30, 0.25]),
        "liquidity_limit_value": np.array([100_000.0, 120_000.0, 80_000.0, 60_000.0]),
        "max_turnover_ratio": 0.10,
        "capital": 1_000_000.0,
    }


def test_portfolio_keyword_interface_and_reuse():
    settings = helix.SolverSettings()
    settings.absolute_tolerance = 1e-7
    settings.relative_tolerance = 1e-7
    optimizer = helix.PortfolioLpOptimizer(settings)

    data = portfolio_data()
    first = optimizer.solve(**data)

    assert first.success
    np.testing.assert_allclose(first.target_weight, [0.26, 0.35, 0.25, 0.14], atol=1e-5)
    np.testing.assert_allclose(first.target_weight.sum(), 1.0, atol=1e-7)
    assert first.turnover_ratio <= data["max_turnover_ratio"] + 1e-7

    data["alpha"] = data["alpha"].copy()
    data["alpha"][2] += 0.01
    second = optimizer.solve(**data)

    assert second.success
    assert second.solver_result.stats.reused_workspace
    assert not second.solver_result.stats.updated_matrices


def test_portfolio_input_object_and_invalid_status():
    data = portfolio_data()
    input_data = helix.PortfolioOptimizationInput()
    for name, value in data.items():
        setattr(input_data, name, value)

    optimizer = helix.PortfolioLpOptimizer()
    assert optimizer.solve(input_data).success

    input_data.capital = -1.0
    invalid = optimizer.solve(input_data)
    assert not invalid.success
    assert invalid.solver_result.status == helix.SolveStatus.INVALID_PROBLEM
