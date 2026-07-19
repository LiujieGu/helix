import numpy as np

import helix


settings = helix.SolverSettings()
settings.absolute_tolerance = 1e-7
settings.relative_tolerance = 1e-7

optimizer = helix.PortfolioLpOptimizer(settings)
result = optimizer.solve(
    alpha=np.array([0.02, 0.08, 0.05, -0.01]),
    current_position_value=np.array([300_000.0, 250_000.0, 250_000.0, 200_000.0]),
    max_position_weight=np.array([0.35, 0.40, 0.30, 0.25]),
    liquidity_limit_value=np.array([100_000.0, 120_000.0, 80_000.0, 60_000.0]),
    max_turnover_ratio=0.10,
    capital=1_000_000.0,
)

if not result.success:
    raise RuntimeError(result.solver_result.message)

print("target weights:", result.target_weight)
print("trade values:", result.trade_value)
print("turnover ratio:", result.turnover_ratio)
