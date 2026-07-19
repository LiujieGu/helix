import numpy as np
from scipy import sparse

import helix


def test_qp_solve_and_workspace_reuse():
    h = sparse.csc_matrix([[4.0, 1.0], [1.0, 2.0]])
    a = sparse.csc_matrix((0, 2))
    lower = np.empty(0)
    upper = np.empty(0)

    solver = helix.OsqpSolver()
    first = solver.solve(h, np.array([-1.0, -1.0]), a, lower, upper)

    assert first.success
    np.testing.assert_allclose(first.x, [1.0 / 7.0, 3.0 / 7.0], atol=1e-5)
    assert not first.stats.reused_workspace

    second = solver.solve(
        H=h,
        c=np.array([-2.0, -1.0]),
        A=a,
        lower=lower,
        upper=upper,
    )

    assert second.success
    np.testing.assert_allclose(second.x, [3.0 / 7.0, 2.0 / 7.0], atol=1e-5)
    assert second.stats.reused_workspace
    assert not second.stats.updated_matrices


def test_lp_accepts_csr_constraints():
    # maximize x[0] + 2*x[1], with sum(x) = 1 and x >= 0
    c = np.array([-1.0, -2.0])
    a = sparse.csr_matrix([[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]])
    lower = np.array([1.0, 0.0, 0.0])
    upper = np.array([1.0, np.inf, np.inf])

    result = helix.LpSolver().solve(c, a, lower, upper)

    assert result.success
    np.testing.assert_allclose(result.x, [0.0, 1.0], atol=1e-5)
    np.testing.assert_allclose(result.objective, -2.0, atol=1e-5)
    assert result.status_name == "solved"
    assert result.stats.primal_residual <= 1e-5


def test_invalid_problem_returns_structured_status():
    result = helix.LpSolver().solve(
        c=np.array([1.0, 2.0]),
        A=sparse.csc_matrix([[1.0, 0.0]]),
        lower=np.array([]),
        upper=np.array([]),
    )

    assert not result.success
    assert result.status == helix.SolveStatus.INVALID_PROBLEM
    assert result.message
