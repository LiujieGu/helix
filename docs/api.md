# Helix 公共 API 参考

本文档记录 Helix 当前对外公开的 C++ 与 Python API。每个接口说明以下内容：

- 接口用途和数学含义；
- 输入参数、维度和约束；
- 内部实现路径；
- 返回结果和失败方式；
- workspace 复用、线程安全等注意事项。

当前项目版本为 `0.1.0`，API 仍处于 alpha 阶段。新增、删除或修改公共接口时，应在同一提交中
更新本文档、测试和示例。

## 1. 通用数学约定

### 1.1 QP 标准形式

Helix 的通用凸二次规划采用：

```text
minimize    0.5 * x' P x + q' x
subject to  l <= A x <= u
```

C++ 中历史命名为：

| 数学符号 | C++ 字段 | Python 字段 | 维度 |
| --- | --- | --- | --- |
| `P` | `QpProblem::H` | `QPProblem.P` | `n × n` |
| `q` | `QpProblem::c` | `QPProblem.q` | `n` |
| `A` | `QpProblem::A` | `QPProblem.A` | `m × n` |
| `l` | `QpProblem::lower` | `QPProblem.l` | `m` |
| `u` | `QpProblem::upper` | `QPProblem.u` | `m` |

要求：

- `P` 必须为半正定矩阵；
- `P` 可以是完整对称矩阵，也可以只存上三角；
- `P`、`q`、`A` 的系数必须有限；
- `l`、`u` 可以包含负无穷或正无穷，但不能包含 NaN；
- 每行必须满足 `l[i] <= u[i]`；
- 所有变量上下界、非负、预算等约束都必须显式写入 `A/l/u`；
- 无约束 QP 使用形状为 `0 × n` 的 `A` 和长度为零的 `l/u`。

### 1.2 LP 标准形式

线性规划使用相同的约束形式：

```text
minimize    c' x
subject to  lower <= A x <= upper
```

内部将其转换为 `P = 0` 的 QP，并通过 OSQP 后端求解。

### 1.3 稀疏矩阵

C++ 公共稀疏矩阵类型为：

```cpp
using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;
```

内部会将矩阵压缩为 CSC。大型问题应直接构造稀疏矩阵，避免先创建大型稠密矩阵。

## 2. 通用状态与结果

定义位置：`src/helix/types.hpp`

### 2.1 `SolveStatus`

表示求解终止状态。

| C++ | Python | 含义 |
| --- | --- | --- |
| `kSolved` | `SOLVED` | 求解成功 |
| `kSolvedInaccurate` | `SOLVED_INACCURATE` | 达到较宽松精度，仍被 `success()` 视为成功 |
| `kPrimalInfeasible` | `PRIMAL_INFEASIBLE` | 原问题不可行 |
| `kDualInfeasible` | `DUAL_INFEASIBLE` | 对偶不可行，通常表示原问题无界 |
| `kMaxIterations` | `MAX_ITERATIONS` | 达到迭代上限 |
| `kTimeLimit` | `TIME_LIMIT` | 达到时间上限 |
| `kNonConvex` | `NON_CONVEX` | 检测到非凸问题 |
| `kInterrupted` | `INTERRUPTED` | 求解被中断 |
| `kInvalidProblem` | `INVALID_PROBLEM` | Helix 输入校验失败 |
| `kSetupFailure` | `SETUP_FAILURE` | OSQP 初始化或数据更新失败 |
| `kUnknown` | `UNKNOWN` | 未映射或未知错误 |

#### C++：`to_string`

```cpp
const char* helix::to_string(SolveStatus status) noexcept;
```

- 输入：一个 `SolveStatus`。
- 实现：使用固定映射返回小写英文状态文本。
- 输出：静态字符串指针，无需释放。
- Python 对应：`SolveResult.status_name`。

### 2.2 `SolverSettings`

构造求解器时传入的配置。

| 字段 | 默认值 | 功能 |
| --- | ---: | --- |
| `absolute_tolerance` | `1e-6` | OSQP 绝对收敛容差 |
| `relative_tolerance` | `1e-6` | OSQP 相对收敛容差 |
| `max_iterations` | `10000` | 最大 ADMM 迭代次数 |
| `time_limit_seconds` | `0.0` | 求解时间限制；零表示禁用 |
| `warm_start` | `true` | 是否保留并使用上一次迭代点 |
| `polish` | `true` | 是否执行 OSQP polishing |
| `verbose` | `false` | 是否输出 OSQP 日志 |

实现路径：

```text
SolverSettings
  -> OsqpSolver::Impl 构造
  -> OSQPSettings
  -> osqp_setup()
```

注意：

- 设置在求解器构造时复制；
- 构造后修改原来的 `SolverSettings` 对象不会影响已有求解器；
- 当前没有运行时修改 settings 的公共接口，需要重新构造求解器。

### 2.3 `SolverStatistics`

只读求解统计信息。

| 字段 | 含义 |
| --- | --- |
| `iterations` | 本次 OSQP 迭代次数 |
| `rho_updates` | 本次自适应 `rho` 更新次数 |
| `objective` | 最终目标函数值 |
| `primal_residual` | 最终原始残差 |
| `dual_residual` | 最终对偶残差 |
| `setup_time_seconds` | OSQP workspace 初始化耗时 |
| `solve_time_seconds` | OSQP 报告的求解耗时 |
| `reused_workspace` | 本次是否复用了已有 workspace |
| `updated_matrices` | 复用 workspace 时是否更新了 `P/A` 数值 |

`objective`、残差在没有有效求解结果时可能为 NaN。时间字段是 OSQP 内部统计，不包含全部
Python/C++ 参数转换和 Helix 输入校验时间。

### 2.4 `SolveResult`

通用 LP/QP 求解结果。

| 字段/属性 | 类型 | 含义 |
| --- | --- | --- |
| `x` | 向量 | 原始变量解，长度为 `n` |
| `y` | 向量 | 线性约束对偶解，长度为 `m` |
| `status` | `SolveStatus` | 结构化终止状态 |
| `stats` | `SolverStatistics` | 求解统计 |
| `message` | 字符串 | 后端状态或输入错误说明 |
| `objective` | Python 只读属性 | `stats.objective` 的快捷访问 |
| `status_name` | Python 只读属性 | `to_string(status)` |
| `success()` / `success` | 布尔值 | 是否为 solved 或 solved inaccurate |

失败时通常不填充 `x/y`。调用方必须先检查 `success()`，不能把空向量当作零解。

## 3. C++ 通用 QP API

需要包含：

```cpp
#include "helix/osqp_solver.hpp"
```

### 3.1 `QpProblem`

```cpp
struct QpProblem {
    SparseMatrix H;
    Eigen::VectorXd c;
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};
```

- 功能：保存一个标准形式的凸 QP。
- 输入：调用方填写全部五个字段。
- 实现：纯数据对象，不在赋值时做校验。
- 输出：由 `OsqpSolver::solve()` 消费。

### 3.2 `QpSolver`

```cpp
class QpSolver {
public:
    virtual ~QpSolver() = default;
    virtual SolveResult solve(const QpProblem& problem) = 0;
    virtual const char* name() const noexcept = 0;
};
```

- 功能：QP 后端抽象接口。
- 当前实现：`OsqpSolver`。
- 用途：领域优化器可以依赖抽象 QP 接口，而不是直接依赖 OSQP。

### 3.3 `OsqpSolver::OsqpSolver`

```cpp
explicit OsqpSolver(SolverSettings settings = {});
```

- 输入：求解设置，省略时使用默认值。
- 实现：创建内部状态并转换为 `OSQPSettings`，此时尚未创建 OSQP workspace。
- 输出：一个可移动、不可复制的求解器对象。
- 线程安全：单个实例不是线程安全的；每个并行线程应持有独立实例。

### 3.4 `OsqpSolver::solve`

```cpp
SolveResult solve(const QpProblem& problem);
```

输入：

- `problem.H`：`n × n` 半正定 Hessian；
- `problem.c`：长度 `n` 的线性目标；
- `problem.A`：`m × n` 约束矩阵；
- `problem.lower/upper`：长度 `m` 的边界。

实现路径：

```text
QpProblem
  -> 维度、有限值、边界和 Hessian 对称性校验
  -> 提取 P 的上三角
  -> P/A 转换为压缩 CSC
  -> 比较维度与稀疏结构
     -> 结构相同：osqp_update_data_mat / osqp_update_data_vec
     -> 结构不同：清理旧 workspace，再执行 osqp_setup
  -> 可选 cold start
  -> osqp_solve
  -> 映射状态、解和统计信息
  -> SolveResult
```

输出：

- 成功：`SolveResult.x/y`、目标值、残差和统计信息；
- 输入错误：`status == kInvalidProblem`，不抛异常；
- OSQP setup/update 错误：`status == kSetupFailure`；
- 不可行、非凸、超时等：对应结构化状态。

workspace 复用：

- 维度及 `H/A` 稀疏结构相同时复用 workspace；
- 只更新 `c/lower/upper` 时不更新矩阵分解；
- `H/A` 数值变化但结构不变时更新矩阵并重新进行必要的数值分解；
- 稀疏结构变化时重新 setup；
- 当前每次 `solve()` 仍会遍历和复制 `H/A`。

### 3.5 `OsqpSolver::warm_start`

```cpp
void warm_start(
    const Eigen::VectorXd& primal = {},
    const Eigen::VectorXd& dual = {});
```

参数：

- `primal`：可选原始变量初值，非空时长度必须为 `n`；
- `dual`：可选对偶变量初值，非空时长度必须为 `m`；
- 两者至少提供一个，元素必须有限。

实现：

```text
检查已有 workspace 和设置
  -> double 向量转换为 OSQPFloat
  -> osqp_warm_start()
  -> 下一次 solve() 使用指定迭代点
```

异常：

- 尚未成功创建 workspace：`std::logic_error`；
- `SolverSettings::warm_start == false`：`std::logic_error`；
- 两个向量都为空、维度错误或存在非有限值：`std::invalid_argument`；
- OSQP 拒绝初值：`std::runtime_error`。

注意：必须至少先调用一次能够创建 workspace 的 `solve()`。

### 3.6 `OsqpSolver::has_workspace`

```cpp
bool has_workspace() const noexcept;
```

- 输入：无。
- 实现：检查内部 OSQP solver 指针。
- 输出：当前是否持有可复用 workspace。

### 3.7 `OsqpSolver::reset`

```cpp
void reset() noexcept;
```

- 输入：无。
- 实现：调用 `osqp_cleanup()`，清空矩阵存储、维度、分解和 warm-start 状态。
- 输出：无。
- 后续行为：下一次 `solve()` 重新 setup。

### 3.8 `OsqpSolver::name`

```cpp
const char* name() const noexcept;
```

返回 `"OSQP"`。

## 4. C++ 通用 LP API

需要包含：

```cpp
#include "helix/lp_solver.hpp"
```

### 4.1 `LpProblem`

```cpp
struct LpProblem {
    Eigen::VectorXd c;
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};
```

- 功能：保存一个标准形式 LP。
- 输入：长度为 `n` 的目标、`m × n` 约束矩阵和长度为 `m` 的边界。
- 实现：纯数据对象。

### 4.2 `LpSolver::LpSolver`

```cpp
explicit LpSolver(SolverSettings settings = {});
```

使用设置构造内部 `OsqpSolver`。

### 4.3 `LpSolver::solve`

```cpp
SolveResult solve(const LpProblem& problem);
```

实现路径：

```text
LpProblem
  -> 构造 n × n 的零 Hessian
  -> 转换为 QpProblem
  -> OsqpSolver::solve()
  -> SolveResult
```

输入和输出规则与 QP 相同。OSQP 是一阶凸优化后端，不是 simplex 法 LP 求解器。

### 4.4 `LpSolver::reset`

```cpp
void reset() noexcept;
```

调用内部 `OsqpSolver::reset()`，清除 workspace。

### 4.5 `LpSolver::name`

```cpp
const char* name() const noexcept;
```

返回 `"LP (OSQP backend)"`。

## 5. C++ 单纯形特例 API

需要包含：

```cpp
#include "helix/simplex.hpp"
```

这些函数目前没有暴露到 Python。

### 5.1 `project_to_simplex`

```cpp
Eigen::VectorXd project_to_simplex(
    const Eigen::Ref<const Eigen::VectorXd>& values,
    double mass = 1.0);
```

功能：

\[
\operatorname*{argmin}_x \frac12\|x-values\|_2^2
\quad\text{s.t.}\quad x\ge0,\ \sum_i x_i=mass
\]

参数：

- `values`：有限、非空向量；
- `mass`：有限正数，默认 `1.0`。

实现：排序后计算阈值并截断，时间复杂度 `O(n log n)`，额外空间 `O(n)`。

输出：长度与输入相同、非负且和为 `mass` 的欧氏投影。

异常：输入为空、存在非有限值或 `mass <= 0` 时抛 `std::invalid_argument`。

### 5.2 `minimize_linear_on_simplex`

```cpp
Eigen::VectorXd minimize_linear_on_simplex(
    const Eigen::Ref<const Eigen::VectorXd>& objective,
    double mass = 1.0);
```

功能：

\[
\operatorname*{argmin}_x objective^T x
\quad\text{s.t.}\quad x\ge0,\ \sum_i x_i=mass
\]

参数：

- `objective`：有限、非空线性目标；
- `mass`：有限正数。

实现：找到最小目标系数，将全部 `mass` 分配给该位置。时间复杂度 `O(n)`。

输出：只有一个非零元素的精确最优解；并列最小时选择第一个索引。

异常：无效输入时抛 `std::invalid_argument`。

## 6. C++ 投资组合 LP API

需要包含：

```cpp
#include "helix/portfolio.hpp"
```

### 6.1 `PortfolioOptimizationInput`

长仓、满仓组合优化输入。

| 字段 | 维度/单位 | 含义 |
| --- | --- | --- |
| `alpha` | `n` | 每只资产的预期收益或目标分数 |
| `current_position_value` | `n`，金额 | 当前持仓金额 |
| `max_position_weight` | `n`，比例 | 单票最大目标权重 |
| `liquidity_limit_value` | `n`，金额 | 单票允许的最大绝对交易金额 |
| `max_turnover_ratio` | 标量，比例 | 最大单边换手率 |
| `capital` | 标量，金额 | 组合总资本 |

要求：

- 所有向量长度相同且非空；
- 所有值有限；
- `capital > 0`；
- 换手率和三个限制类向量不得为负；
- `current_position_value.sum()` 必须近似等于 `capital`。

### 6.2 `PortfolioConstraintSystem`

```cpp
struct PortfolioConstraintSystem {
    SparseMatrix A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};
```

保存投资组合 LP/QP 可共享的线性约束系统。变量排列为：

```text
[target_weight(0:n), absolute_trade_weight(0:n)]
```

因此变量数为 `2n`，约束数为 `4n + 2`。

### 6.3 `make_portfolio_constraints`

```cpp
PortfolioConstraintSystem make_portfolio_constraints(
    const PortfolioOptimizationInput& input);
```

构造：

- `sum(target_weight) == 1`；
- `target_weight >= 0`；
- 单票目标权重上限；
- 单票流动性交易限制；
- 辅助变量满足绝对交易量上下界；
- `sum(abs(target-current)) / (2*capital) <= max_turnover_ratio`。

实现会提前检查仓位上限与流动性限制下是否仍可能达到满仓。

输出：可以直接赋给 `LpProblem` 或 `QpProblem` 的 `A/lower/upper`。

异常：输入格式错误或约束显然不可行时抛 `std::invalid_argument`。

### 6.4 `PortfolioOptimizationResult`

| 字段 | 含义 |
| --- | --- |
| `target_position_value` | 最优目标持仓金额 |
| `target_weight` | 最优目标权重 |
| `trade_value` | `target_position_value - current_position_value` |
| `expected_alpha` | `alpha.dot(target_weight)` |
| `turnover_ratio` | `sum(abs(trade_value)) / (2*capital)` |
| `solver_result` | 底层 `SolveResult` |
| `success()` | 转发 `solver_result.success()` |

失败时只有 `solver_result` 保证有效，其他领域结果保持默认值或空向量。

### 6.5 `make_portfolio_result`

```cpp
PortfolioOptimizationResult make_portfolio_result(
    const PortfolioOptimizationInput& input,
    SolveResult solve_result);
```

- 输入：原始投资组合输入和一个底层求解结果；
- 实现：成功时读取求解变量前 `n` 项作为目标权重，并计算金额、交易、alpha 和换手率；
- 输出：`PortfolioOptimizationResult`；
- 失败时：保留底层失败结果，不转换领域字段。

### 6.6 `PortfolioLpOptimizer`

#### 构造

```cpp
explicit PortfolioLpOptimizer(SolverSettings settings = {});
```

使用设置创建一个可复用 `LpSolver`。

#### `solve`

```cpp
PortfolioOptimizationResult solve(
    const PortfolioOptimizationInput& input);
```

目标：

```text
maximize alpha' target_weight
```

内部实现路径：

```text
PortfolioOptimizationInput
  -> make_portfolio_constraints()
  -> 构造变量 [target_weight, absolute_trade_weight]
  -> LP 线性目标 [-alpha, 0]
  -> LpSolver::solve()
  -> make_portfolio_result()
  -> PortfolioOptimizationResult
```

输入校验异常会被捕获并转换为：

```text
solver_result.status = kInvalidProblem
solver_result.message = 具体原因
```

不会从 `solve()` 向调用方抛出 `std::invalid_argument`。

#### `reset`

```cpp
void reset() noexcept;
```

清除内部 LP/OSQP workspace。

### 6.7 `optimize_portfolio_lp`

```cpp
PortfolioOptimizationResult optimize_portfolio_lp(
    const PortfolioOptimizationInput& input,
    SolverSettings settings = {});
```

- 功能：一次性长仓投资组合 LP 优化；
- 实现：临时构造 `PortfolioLpOptimizer` 并调用 `solve()`；
- 输出：`PortfolioOptimizationResult`；
- 注意：函数返回后 workspace 被销毁，批量重复优化应复用 `PortfolioLpOptimizer`。

## 7. Python 通用类型

Python 模块名为 `helix`，最低支持 Python 3.9。向量应传入一维 NumPy 数组。

### 7.1 `helix.__version__`

模块版本字符串，由 CMake 项目版本在编译时写入。当前值为 `"0.1.0"`。

### 7.2 `helix.SolverSettings`

```python
settings = helix.SolverSettings()
settings.absolute_tolerance = 1e-7
settings.relative_tolerance = 1e-7
settings.max_iterations = 20_000
settings.time_limit_seconds = 0.5
settings.warm_start = True
settings.polish = True
settings.verbose = False
```

字段含义与 C++ `SolverSettings` 相同。必须在构造 solver/optimizer 前完成设置。

### 7.3 `helix.SolveStatus`

Python 枚举值见第 2.1 节。

### 7.4 `helix.SolverStatistics`

所有属性只读，字段见第 2.3 节。

### 7.5 `helix.SolveResult`

所有属性只读，字段见第 2.4 节：

```python
if result.success:
    x = result.x
else:
    raise RuntimeError(f"{result.status_name}: {result.message}")
```

## 8. Python 通用 QP API

### 8.1 `helix.QPProblem`

```python
problem = helix.QPProblem(P=P, q=q, A=A, l=l, u=u)
```

参数：

| 参数 | 接受类型 | 形状 |
| --- | --- | --- |
| `P` | 二维 NumPy 数组、SciPy CSC/CSR 等稀疏矩阵 | `(n, n)` |
| `q` | 一维 NumPy 数组 | `(n,)` |
| `A` | 二维 NumPy 数组、SciPy CSC/CSR 等稀疏矩阵 | `(m, n)` |
| `l` | 一维 NumPy 数组 | `(m,)` |
| `u` | 一维 NumPy 数组 | `(m,)` |

功能：持有一个标准 QP。属性 `P/q/A/l/u` 可重新赋值。

实现：

- 稀疏输入通过 pybind11 Eigen caster 转为 `SparseMatrix`；
- 稠密输入先转为 `Eigen::MatrixXd`，再转为稀疏矩阵；
- 真正的维度、有限值和边界校验在 `solve()` 中执行。

别名：`helix.QpProblem` 与 `helix.QPProblem` 是同一个类。

### 8.2 `helix.QPSolver`

#### 构造

```python
solver = helix.QPSolver(settings=helix.SolverSettings())
```

参数：可选 `SolverSettings`。

实现：持有一个 C++ `OsqpSolver`。`helix.OsqpSolver` 是兼容别名。

#### `solve(problem)`

```python
result = solver.solve(problem)
```

- 输入：`QPProblem`；
- 实现：释放 Python GIL 后调用 `OsqpSolver::solve()`；
- 输出：`SolveResult`；
- 复用：同一 solver 重复调用时按 C++ 规则复用 workspace。

#### `solve(P, q, A, l, u)`

```python
result = solver.solve(P=P, q=q, A=A, l=l, u=u)
```

- 参数类型与 `QPProblem` 相同；
- 内部临时构造 `QpProblem` 后调用同一求解路径；
- 兼容旧关键字：`H/c/A/lower/upper`。

#### `warm_start`

```python
solver.warm_start(x=primal)
solver.warm_start(y=dual)
solver.warm_start(x=primal, y=dual)
```

- `x`：可选长度 `n` 的有限 NumPy 向量；
- `y`：可选长度 `m` 的有限 NumPy 向量；
- 至少提供一个；
- 必须先通过 `solve()` 创建 workspace；
- C++ 异常在 Python 中映射为 `RuntimeError` 或 `ValueError`。

#### `has_workspace`

```python
solver.has_workspace  # bool
```

只读属性，表示当前是否持有 C++ OSQP workspace。

#### `reset`

```python
solver.reset()
```

清除 workspace 和 warm-start 状态。

#### `name`

```python
solver.name  # "OSQP"
```

只读后端名称。

### 8.3 `helix.solve_qp`

```python
result = helix.solve_qp(
    P=P,
    q=q,
    A=A,
    l=l,
    u=u,
    settings=settings,
)
```

参数：

- `P/q/A/l/u`：同 `QPProblem`；
- `settings`：可选 `SolverSettings`。

实现：

```text
Python 参数
  -> 临时 QpProblem
  -> 临时 OsqpSolver
  -> 释放 GIL并求解
  -> SolveResult
```

输出：`SolveResult`。

注意：适合一次性问题；每次调用都会重新创建和销毁 workspace。重复求解应使用 `QPSolver`。

## 9. Python LP API

### 9.1 `helix.LpSolver`

#### 构造

```python
solver = helix.LpSolver(settings=helix.SolverSettings())
```

#### `solve`

```python
result = solver.solve(
    c=c,
    A=A,
    lower=lower,
    upper=upper,
)
```

参数：

- `c`：长度 `n` 的 NumPy 向量；
- `A`：形状 `(m, n)` 的 SciPy 稀疏矩阵；
- `lower/upper`：长度 `m` 的 NumPy 向量。

当前 Python LP 绑定要求 `A` 为 SciPy 稀疏矩阵；它还没有使用 QP 接口的稠密矩阵转换层。

实现：

```text
Python 参数
  -> LpProblem
  -> 释放 GIL
  -> C++ LpSolver
  -> 零 Hessian QP
  -> OSQP
  -> SolveResult
```

#### `reset`

清除内部 workspace。

#### `name`

返回 `"LP (OSQP backend)"`。

## 10. Python 投资组合 API

### 10.1 `helix.PortfolioOptimizationInput`

```python
data = helix.PortfolioOptimizationInput()
data.alpha = alpha
data.current_position_value = current_position_value
data.max_position_weight = max_position_weight
data.liquidity_limit_value = liquidity_limit_value
data.max_turnover_ratio = max_turnover_ratio
data.capital = capital
```

字段、单位和约束见第 6.1 节。

### 10.2 `helix.PortfolioOptimizationResult`

只读属性：

- `target_position_value`；
- `target_weight`；
- `trade_value`；
- `expected_alpha`；
- `turnover_ratio`；
- `solver_result`；
- `success`。

含义见第 6.4 节。

### 10.3 `helix.PortfolioLpOptimizer`

#### 构造

```python
optimizer = helix.PortfolioLpOptimizer(settings=helix.SolverSettings())
```

#### `solve(input)`

```python
result = optimizer.solve(data)
```

输入：`PortfolioOptimizationInput`。

#### `solve(...)`

```python
result = optimizer.solve(
    alpha=alpha,
    current_position_value=current_position_value,
    max_position_weight=max_position_weight,
    liquidity_limit_value=liquidity_limit_value,
    max_turnover_ratio=max_turnover_ratio,
    capital=capital,
)
```

输入：与数据对象相同的六个字段。

两种重载都释放 Python GIL，调用同一个 C++ `PortfolioLpOptimizer::solve()`，返回
`PortfolioOptimizationResult`。重复调用同一对象可以复用 workspace。

#### `reset`

清除内部 workspace。

### 10.4 `helix.optimize_portfolio_lp`

```python
result = helix.optimize_portfolio_lp(
    alpha=alpha,
    current_position_value=current_position_value,
    max_position_weight=max_position_weight,
    liquidity_limit_value=liquidity_limit_value,
    max_turnover_ratio=max_turnover_ratio,
    capital=capital,
    settings=settings,
)
```

- 功能：一次性长仓、满仓投资组合 LP；
- 输入：六个投资组合字段和可选 settings；
- 实现：临时构造 `PortfolioLpOptimizer` 并释放 GIL 求解；
- 输出：`PortfolioOptimizationResult`；
- 注意：不会跨调用复用 workspace。

## 11. 错误处理规范

### 11.1 通用求解

数学问题无效、不可行、无界或求解失败时，优先通过 `SolveResult.status/message` 返回，不依赖异常：

```cpp
helix::SolveResult result = solver.solve(problem);
if (!result.success()) {
    // 检查 result.status 和 result.message
}
```

```python
result = solver.solve(problem)
if not result.success:
    raise RuntimeError(f"{result.status_name}: {result.message}")
```

### 11.2 API 使用错误

求解过程之外的错误，例如显式 warm start 的调用顺序、向量维度或非有限值，使用异常报告。

### 11.3 `SOLVED_INACCURATE`

`SOLVED_INACCURATE` 被统一视为成功。精度敏感的调用方应额外检查：

- `result.status`；
- `result.stats.primal_residual`；
- `result.stats.dual_residual`。

## 12. 线程与生命周期

- 一个 solver/optimizer 实例保存可变 workspace，不能被多个线程同时调用；
- 并行求解应为每个线程创建独立实例；
- Python 求解期间释放 GIL，因此不同线程持有不同 solver 时可以并行进入 C++；
- `SolveResult` 保存自己的解向量和统计信息，不依赖 solver 的后续生命周期；
- Python `result.stats` 和 `portfolio_result.solver_result` 是父结果对象的内部引用，不应在父对象
  销毁后继续持有。

## 13. 公共 API 维护规范

修改公共接口时，提交必须同步检查：

1. `src/helix/*.hpp` 的签名和注释；
2. `python/bindings.cpp` 是否需要同步暴露；
3. 本文档的参数、返回值和实现路径；
4. `README.md` 的入口示例；
5. C++ 与 Python 测试；
6. `examples/` 中至少一个可运行示例；
7. 是否破坏已有别名、关键字参数或状态语义。

如果只修改内部实现但外部行为没有变化，也应检查本文档中的“实现路径”和性能说明是否仍然准确。
