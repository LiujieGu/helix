# helix

Helix 是一个 C++20 数值优化库。目前提供：

- 通用凸 QP：`min 0.5 x' H x + c' x`，约束为 `lower <= A x <= upper`；
- 使用同一约束模型的 LP；
- 概率单纯形投影和单纯形线性目标的精确解；
- 基于 OSQP 的稀疏求解、warm start 和固定稀疏结构 workspace 复用。

所有约束都必须显式传入，求解器不会隐式添加 `sum(x)=1` 或 `x>=0`。

## 项目定位与成熟度

Helix 目前是一个基于成熟 OSQP 后端、处于 alpha 阶段的稀疏凸 LP/QP 求解器。现有接口、
输入校验、状态诊断和投资组合示例已经可以支持研究、回测及内部策略开发，但尚未经过足够的
大规模交叉验证和生产压力测试，因此暂不承诺生产级稳定性或极致性能。

当前可以确认：

- 示例和测试覆盖的 LP/QP 能够得到满足约束、数学含义正确的结果；
- 固定矩阵结构、重复更新目标和边界时可以复用 workspace、矩阵分解和 warm start；
- 求解失败时能够区分无效输入、不可行、非凸、迭代上限和精度不足等状态。

当前仍需继续建设：

- 与其他成熟求解器进行随机问题和数值精度交叉验证；
- 病态、奇异、不可行及大型稀疏问题的压力测试；
- ASan、UBSan、长期重复更新和内存稳定性检查；
- 持续 benchmark、性能回归与明确的 API 兼容策略。

因此，当前更适合将 Helix 描述为“可用且结构正确的工程原型”，而不是已经完成生产验证的
通用高性能求解器。

公共 C++ 与 Python API 的参数、返回值、内部实现路径和错误语义统一记录在
[`docs/api.md`](docs/api.md)。新增或修改公共接口时应同步更新该文档。

## 构建与测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/examples/portfolio/helix_portfolio_qp
```

### Python 扩展

Helix 可以构建为接受 NumPy 向量和 SciPy 稀疏矩阵的 Python 扩展模块：
构建时需要 Python 3.9 以上版本及对应的开发头文件（Debian/Ubuntu 中为 `python3-dev`）。

```bash
python3 -m venv .venv
.venv/bin/pip install .
.venv/bin/python examples/python/portfolio_lp.py
.venv/bin/python -m pytest tests/python
```

也可以直接用 CMake 构建，普通 C++ 构建默认不会引入 Python 依赖：

```bash
cmake -S . -B build-python -DHELIX_BUILD_PYTHON=ON
cmake --build build-python -j
PYTHONPATH=build-python/python python3 -c "import helix; print(helix.__version__)"
```

Python 通用 QP 使用与 OSQP 一致的标准形式：

```text
minimize    0.5 * x.T @ P @ x + q.T @ x
subject to  l <= A @ x <= u
```

```python
import helix
import numpy as np
from scipy import sparse

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
    raise RuntimeError(f"{result.status_name}: {result.message}")

print(result.x, result.objective)
```

`P` 和 `A` 接受二维 NumPy 数组以及 SciPy CSC/CSR 稀疏矩阵；大型问题应使用稀疏矩阵。
`P` 必须半正定，可以传完整对称矩阵或只传上三角。`l`、`u` 支持无穷边界。Helix 不会隐式
添加非负、预算或变量上下界约束，这些约束都需要作为 `A/l/u` 的行显式传入。

重复调用同一个 `QPSolver` 会保留 C++ workspace 和 warm-start 状态。保持 `P`、`A` 的稀疏
结构不变时，Helix 会在原 workspace 中更新数据；也可以用 `solver.warm_start(x=..., y=...)`
显式提供初始点。一次性问题可以调用：

```python
result = helix.solve_qp(P=P, q=q, A=A, l=l, u=u)
```

`helix.QPSolver` 当前使用 OSQP 后端；`helix.OsqpSolver` 是它的兼容名称。通用 LP 仍可通过
`helix.LpSolver` 调用。完整 QP 示例位于 `examples/python/qp.py`。

依赖 Eigen 3.4 和 OSQP 1.0，由 CMake `FetchContent` 获取。依赖项目的测试、示例和文档默认关闭。

投资组合示例包括：

- `examples/portfolio/Portfolio_LP.cpp`：最大化 alpha，限制单票仓位、流动性和组合换手率；
- `examples/portfolio/Portfolio_QP.cpp`：带协方差风险惩罚的均值–方差 QP，并输出组合波动率和
  Sharpe。

运行 LP 示例：

```bash
./build/examples/portfolio/helix_portfolio_lp
```

LP 投资组合优化已经作为公共 API 打包。重复优化时建议复用同一个 optimizer：

```cpp
#include "helix/portfolio.hpp"

helix::PortfolioOptimizationInput input;
input.alpha = alpha;
input.current_position_value = current_position;
input.max_position_weight = max_position_weight;
input.liquidity_limit_value = liquidity_limit_value;
input.max_turnover_ratio = 0.10;
input.capital = 1'000'000.0;

helix::PortfolioLpOptimizer optimizer;
helix::PortfolioOptimizationResult result = optimizer.solve(input);
if (result.success()) {
    // result.target_position_value: 最优目标持仓金额
    // result.target_weight:         最优目标权重
    // result.trade_value:           需要执行的交易金额
    // result.turnover_ratio:        实际单边换手率
}
```

`current_position_value`、`liquidity_limit_value` 和 `capital` 必须使用相同金额单位；
`max_position_weight` 和 `max_turnover_ratio` 使用比例。当前单边换手率定义为
`sum(abs(target-current)) / (2*capital)`。一次性调用也可以使用
`optimize_portfolio_lp(input)`，但不会跨调用复用 workspace 和 warm start。

运行均值–方差 QP 示例：

```bash
./build/examples/portfolio/helix_portfolio_qp
```

QP 示例求解：

```text
maximize alpha' * weight
       - 0.5 * risk_aversion * weight' * covariance * weight
```

`risk_aversion` 越大，优化器越重视降低协方差风险；越小，结果越接近纯 alpha LP。示例会计算
最终组合波动率和 Sharpe 比率。需要注意，均值–方差目标是在有效前沿上选择一个点，并不等于
一次 QP 直接最大化 `(return - risk_free) / volatility`。精确最大 Sharpe 是分式/二阶锥问题，
需要 SOCP 求解器或沿有效前沿搜索多个 `risk_aversion`。

## QP 示例

```cpp
#include "helix/osqp_solver.hpp"

helix::QpProblem problem;
problem.H = h.sparseView();
problem.c = c;
problem.A = a.sparseView();
problem.lower = lower;
problem.upper = upper;

helix::OsqpSolver solver;
helix::SolveResult result = solver.solve(problem);
if (!result.success()) {
    // result.status 和 result.message 会区分不可行、非凸、迭代上限等状态。
}
```

`H` 可以是完整对称稀疏矩阵，也可以只存储上三角。`H` 必须为半正定矩阵。重复调用同一个
`OsqpSolver` 时，如果 `H` 和 `A` 的稀疏结构没有变化，Helix 会更新数值并复用 OSQP workspace。

无约束问题需要显式设置 `A.resize(0, n)`，并将 `lower`、`upper` 设为空向量。

## LP 与单纯形特例

通用 LP 使用 `LpProblem` 和 `LpSolver`。对于

```text
minimize c'x
subject to sum(x) = mass, x >= 0
```

直接调用 `minimize_linear_on_simplex(c, mass)`，它会在 `O(n)` 时间内返回精确解，不需要迭代。

## 求解结果

`SolveResult` 包括：

- 原始解 `x` 和对偶解 `y`；
- 结构化 `SolveStatus`；
- 目标值、原始/对偶残差、迭代数和求解耗时；
- `reused_workspace` 和 `updated_matrices`，用于确认重复求解是否复用了 workspace，以及是否
  触发矩阵更新/重新数值分解。

`SolvedInaccurate` 的 `success()` 也为真，调用方可以根据精度要求单独检查 `status` 和残差。

## 性能注意事项

- 优先直接构造 `Eigen::SparseMatrix`，避免先生成大型稠密矩阵；
- 批量重复求解时保持矩阵稀疏结构不变，只更新数值、目标和边界；
- 一个 solver 实例保存可变 workspace，不应被多个线程同时调用；每个线程使用独立实例。

### 实测速度

以下结果来自 AMD Ryzen 9 4900HS、GCC 13.3、Release `-O3` 和 OSQP builtin 后端。
测试问题使用对角 Hessian、一个预算等式和逐变量非负约束。重复求解只修改线性目标 `c`，
保持 `H`、`A` 及其稀疏结构不变。各次求解均使用 25 次 OSQP 迭代。

| 变量数 | 首次求解 | 热启动重复求解中位数 | 热启动吞吐量 |
| ---: | ---: | ---: | ---: |
| 100 | 0.239 ms | 0.073 ms | 约 13,700 次/秒 |
| 1,000 | 2.326 ms | 0.422 ms | 约 2,370 次/秒 |
| 5,000 | 16.230 ms | 2.099 ms | 约 476 次/秒 |

计时包含 Helix 的参数检查、稀疏矩阵遍历、数据更新和 OSQP 求解，不只是 OSQP 内部时间。
这些数字用于说明量级，不是跨硬件性能承诺。病态 Hessian、高度相关的约束、更严格的容差或
质量较差的 warm start 都可能令迭代次数从数十次上升到数百甚至数千次，求解时间通常会随
迭代次数近似线性增长。

### QP 时间和空间复杂度

记：

- `n` 为变量数，`m` 为约束数；
- `K` 为 ADMM 迭代次数；
- `nnz(H)`、`nnz(A)` 为矩阵非零元素数；
- `nnz(L)` 为 KKT 矩阵稀疏 LDL 分解因子的非零元素数。

首次求解包含三部分：

1. Helix 校验、压缩和 CSC 数据准备：`O(nnz(H) + nnz(A))`；
2. KKT 稀疏分解：取决于矩阵拓扑和 fill-in，不能只由 `n`、`m` 精确描述；
3. 每次 ADMM 迭代：约 `O(nnz(H) + nnz(A) + nnz(L))`。

因此首次求解可近似表示为：

```text
T_first = T_factorization
        + O(K * (nnz(H) + nnz(A) + nnz(L)))
```

在完全稠密的最坏情况下，令 `d = n + m`，时间复杂度约为
`O(d^3 + K*d^2)`，空间复杂度为 `O(d^2)`。对于结构良好的稀疏问题，实际开销通常远低于
这个上界，主要取决于排序后的 KKT 分解 fill-in。

稀疏情况下的主要内存复杂度为：

```text
O(nnz(H) + nnz(A) + nnz(L))
```

### 重复求解的三条路径

1. **只修改 `c/lower/upper`**：复用 KKT 分解和 warm-start 状态。OSQP 数据更新约为
   `O(n + m)`，随后主要支付 `K` 次三角求解和稀疏矩阵运算。
2. **矩阵数值变化、稀疏结构不变**：复用 workspace 和符号结构，但必须更新矩阵并重新进行
   数值分解。`SolveResult::stats.updated_matrices` 为 `true`。
3. **矩阵稀疏结构变化**：重新执行完整 setup，成本接近首次求解。

当前 `solve(problem)` 即使只修改 `c/lower/upper`，仍会遍历并复制 `H`、`A`，以确认矩阵结构
和数值没有变化，因此包装层仍有 `O(nnz(H) + nnz(A))` 开销。对于极低延迟批量求解，后续可
增加专用的 `update_linear_cost()`、`update_bounds()` 和无矩阵检查的 `solve()` 接口，进一步
降低热启动延迟。

### LP 和单纯形算法复杂度

通用 `LpSolver` 使用同一 OSQP 后端，除 `H=0` 外，其复杂度和上述 QP 分析相同。OSQP 是
一阶方法，适合稀疏、允许一阶精度和需要重复求解的 LP；它不是专门的 simplex 或
interior-point LP 求解器。

对于概率单纯形上的线性问题，`minimize_linear_on_simplex()` 使用精确特例算法：

- 时间复杂度：`O(n)`；
- 计算过程额外空间：`O(1)`；
- 返回长度为 `n` 的结果向量需要 `O(n)` 空间；
- 无迭代误差。

`project_to_simplex()` 使用排序投影：

- 时间复杂度：`O(n log n)`；
- 空间复杂度：`O(n)`。
