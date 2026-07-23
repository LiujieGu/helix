"""Solve and efficiently re-solve a generic convex quadratic program."""

import numpy as np
from scipy import sparse

import helix


# minimize 0.5*x.T@P@x + q.T@x
# subject to sum(x) = 1 and x >= 0
problem = helix.QPProblem(
    P=sparse.csc_matrix([[4.0, 1.0], [1.0, 2.0]]),
    q=np.array([-1.0, -1.0]),
    A=sparse.csc_matrix([[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]]),
    l=np.array([1.0, 0.0, 0.0]),
    u=np.array([1.0, np.inf, np.inf]),
)

solver = helix.QPSolver()
result = solver.solve(problem)
if not result.success:
    raise RuntimeError(f"QP failed: {result.status_name}: {result.message}")

print("x:", result.x)
print("objective:", result.objective)

# Updating vectors while preserving matrix sparsity reuses the native OSQP workspace.
problem.q = np.array([-2.0, -1.0])
updated = solver.solve(problem)
if not updated.success:
    raise RuntimeError(f"QP update failed: {updated.status_name}: {updated.message}")

print("updated x:", updated.x)
print("reused workspace:", updated.stats.reused_workspace)
