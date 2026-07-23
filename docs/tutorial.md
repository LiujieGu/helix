# Helix 通用优化建模与调用教程

Helix 不只可以用于投资组合仓位设计。投资组合模块只是通用求解器的一个上层应用；底层
`OsqpSolver` 可以求解一般的凸二次规划（QP），`LpSolver` 可以求解采用相同约束形式的线性
规划（LP）。

当一个业务问题可以描述为“最大化或最小化一个目标，同时满足一些限制条件”时，使用 Helix
的主要工作是：定义决策变量，然后把业务目标和限制转换为矩阵。

## 1. Helix 求解的问题

通用 QP 的标准形式为：

```text
minimize    0.5 x' H x + c' x
subject to  lower <= A x <= upper
```

其中：

- `x` 是求解器需要决定的变量；
- `H` 是二次目标矩阵，必须是半正定矩阵；
- `c` 是线性目标系数；
- `A` 是约束矩阵；
- `lower` 和 `upper` 是每一行约束的下界和上界。

LP 是 `H = 0` 的特例：

```text
minimize    c' x
subject to  lower <= A x <= upper
```

Helix 不会隐式增加任何约束。即使业务要求 `x >= 0`，也必须将它显式写入 `A`、`lower` 和
`upper`。

## 2. 定义决策变量

首先明确求解器究竟需要决定什么。例如在生产计划中：

```text
x[0] = 产品 A 的产量
x[1] = 产品 B 的产量
x[2] = 产品 C 的产量
```

求解成功后，`SolveResult::x` 中保存这些变量的最优值。Helix 并不理解“产品”“仓位”或
“机器工时”等业务概念，变量的含义和单位由调用方定义。

同一个模型中的量应采用一致的单位。量级相差过大也可能造成数值问题，例如不要在没有缩放的
情况下同时使用 `1e-9` 和 `1e12` 量级的系数。

## 3. 将最大化转换为最小化

OSQP 的标准形式是最小化。如果业务目标是：

```text
maximize p' x
```

需要取负号，转换成：

```text
minimize -p' x
```

因此代码中应设置：

```cpp
problem.c = -profit;
```

如果最大化目标中包含二次惩罚：

```text
maximize p' x - 0.5 * lambda * x' Q x
```

则转换为：

```text
minimize 0.5 * x' (lambda Q) x - p' x
```

对应：

```cpp
problem.H = (lambda * Q).sparseView();
problem.c = -profit;
```

此时 `lambda * Q` 必须半正定。也就是说，原最大化目标必须是凹二次函数，才能作为凸优化问题
交给当前的 Helix/OSQP 求解。

## 4. 将约束转换为矩阵行

Helix 统一使用 `lower <= A*x <= upper`。每个业务约束通常对应 `A` 中的一行：

| 业务约束 | `lower` | `upper` |
| --- | ---: | ---: |
| `a'x <= b` | `-infinity` | `b` |
| `a'x >= b` | `b` | `+infinity` |
| `a'x = b` | `b` | `b` |
| `l <= a'x <= u` | `l` | `u` |
| `x[i] >= 0` | `0` | `+infinity` |
| `0 <= x[i] <= cap[i]` | `0` | `cap[i]` |

变量边界也通过约束行表达。比如 `0 <= x[0] <= 40`，就在 `A` 中增加一行，该行只有第 0 列
为 `1`，并将对应的上下界设为 `0` 和 `40`。

## 5. 完整 LP 示例：生产计划

假设需要决定两种产品的产量：

```text
maximize    5*x[0] + 4*x[1]

subject to  2*x[0] +   x[1] <= 100   原材料限制
              x[0] + 2*x[1] <= 80    机器工时限制
            0 <= x[0] <= 40
            0 <= x[1] <= 50
```

完整调用代码如下：

```cpp
#include "helix/lp_solver.hpp"

#include <Eigen/Sparse>

#include <iostream>
#include <limits>
#include <vector>

int main() {
    constexpr Eigen::Index variables = 2;
    constexpr Eigen::Index constraints = 4;
    const double infinity = std::numeric_limits<double>::infinity();

    helix::LpProblem problem;

    // maximize 5*x[0] + 4*x[1]
    // 等价于 minimize -5*x[0] - 4*x[1]
    problem.c.resize(variables);
    problem.c << -5.0, -4.0;

    problem.A.resize(constraints, variables);
    std::vector<Eigen::Triplet<double>> entries;

    // 第 0 行：2*x[0] + x[1] <= 100
    entries.emplace_back(0, 0, 2.0);
    entries.emplace_back(0, 1, 1.0);

    // 第 1 行：x[0] + 2*x[1] <= 80
    entries.emplace_back(1, 0, 1.0);
    entries.emplace_back(1, 1, 2.0);

    // 第 2 行：0 <= x[0] <= 40
    entries.emplace_back(2, 0, 1.0);

    // 第 3 行：0 <= x[1] <= 50
    entries.emplace_back(3, 1, 1.0);

    problem.A.setFromTriplets(entries.begin(), entries.end());

    problem.lower.resize(constraints);
    problem.upper.resize(constraints);
    problem.lower << -infinity, -infinity, 0.0, 0.0;
    problem.upper << 100.0, 80.0, 40.0, 50.0;

    helix::SolverSettings settings;
    settings.absolute_tolerance = 1e-7;
    settings.relative_tolerance = 1e-7;
    settings.polish = true;

    helix::LpSolver solver(settings);
    const helix::SolveResult result = solver.solve(problem);

    if (!result.success()) {
        std::cerr << "求解失败: " << result.message << '\n';
        return 1;
    }

    std::cout << "x[0] = " << result.x[0] << '\n';
    std::cout << "x[1] = " << result.x[1] << '\n';

    // stats.objective 是取负后的最小化目标值，取反得到最大利润。
    std::cout << "最大利润 = " << -result.stats.objective << '\n';
}
```

使用 CMake 时，将程序链接到公共目标即可：

```cmake
target_link_libraries(my_optimizer PRIVATE helix::helix)
```

## 6. 通用 QP 调用

假设希望获得较高收益，同时限制方案的风险或波动：

```text
maximize expected_return' x - 0.5 * lambda * x' Q x
```

可以直接构造 `QpProblem`：

```cpp
#include "helix/osqp_solver.hpp"

helix::QpProblem problem;
problem.H = (lambda * Q).sparseView();
problem.c = -expected_return;
problem.A = constraints;
problem.lower = lower;
problem.upper = upper;

helix::OsqpSolver solver;
helix::SolveResult result = solver.solve(problem);
```

二次矩阵 `Q` 不只可以表示投资组合协方差，还可以表示：

- 生产计划的调整成本；
- 控制量变化惩罚；
- 配置偏离参考方案的惩罚；
- 相邻变量之间的不平滑程度。

例如，希望方案不要过度偏离参考值 `x_ref`：

```text
minimize 0.5 * lambda * ||x - x_ref||^2 - profit' x
```

展开并去掉不影响最优解的常数项后：

```text
H = lambda * I
c = -lambda * x_ref - profit
```

## 7. 检查求解结果

不要只读取 `result.x`，应首先检查状态：

```cpp
const helix::SolveResult result = solver.solve(problem);
if (!result.success()) {
    std::cerr << helix::to_string(result.status) << ": " << result.message << '\n';
    return;
}
```

`success()` 对 `Solved` 和 `SolvedInaccurate` 都返回 `true`。对精度敏感的业务还应单独检查：

```cpp
result.status;
result.stats.primal_residual;
result.stats.dual_residual;
result.stats.iterations;
```

若需要严格区分低精度解，可以要求：

```cpp
if (result.status != helix::SolveStatus::kSolved) {
    // 按业务要求拒绝低精度结果或降级处理。
}
```

## 8. 推荐的业务模块结构

不建议将生产计划、资源分配等业务逻辑直接添加到 `OsqpSolver` 中。可以沿用投资组合模块的
分层方式：

```text
业务输入
  -> 建模器：构造 H、c、A、lower、upper
  -> OsqpSolver 或 LpSolver
  -> 业务结果：将 result.x 转回业务对象
```

例如生产优化可以定义：

```cpp
struct ProductionInput {
    Eigen::VectorXd unit_profit;
    Eigen::VectorXd max_production;
    helix::SparseMatrix resource_consumption;
    Eigen::VectorXd resource_capacity;
};

struct ProductionResult {
    Eigen::VectorXd production;
    double total_profit{0.0};
    helix::SolveResult solver_result;

    [[nodiscard]] bool success() const noexcept {
        return solver_result.success();
    }
};

class ProductionOptimizer {
public:
    explicit ProductionOptimizer(helix::SolverSettings settings = {}) : solver_(settings) {}

    ProductionResult solve(const ProductionInput& input);

private:
    helix::LpSolver solver_;
};
```

建模和结果转换可以分别放在 `src/helix/production.hpp` 与
`src/helix/production.cpp`。底层求解器只处理通用数学问题，不需要知道业务概念。

## 9. 重复求解与性能

如果需要在每个回测时点、每个控制周期或每批数据上求解，应长期复用同一个 solver：

```cpp
helix::OsqpSolver solver(settings);

for (...) {
    helix::QpProblem problem = make_problem(...);
    helix::SolveResult result = solver.solve(problem);
}
```

当 `H` 和 `A` 的维度及稀疏结构不变时，Helix 可以复用 OSQP workspace 和 warm start：

```cpp
result.stats.reused_workspace;
result.stats.updated_matrices;
```

重复求解时推荐：

- 保持变量和约束的排列顺序不变；
- 保持 `H` 和 `A` 的稀疏结构不变；
- 只更新目标 `c`、约束边界或现有非零位置的数值；
- 大型问题直接使用 `Eigen::SparseMatrix` 和 triplet 构建，不要先构造大型稠密矩阵；
- 不要在多个线程中共享同一个 solver，每个线程使用独立实例。

即使 workspace 得到复用，当前 `solve()` 仍会遍历和复制 `H`、`A` 来检查结构与数值是否变化。
因此极低延迟场景仍会有一层包装开销。

## 10. 适用范围与限制

当前 Helix 适合：

- 连续变量的线性规划；
- 连续变量的凸二次规划；
- 线性等式和不等式约束；
- 稀疏问题；
- 矩阵结构固定的大量重复求解。

当前不能直接处理：

- 整数变量或 0/1 变量；
- “十个方案中必须选择三个”一类组合约束；
- 非凸二次目标；
- 一般非线性约束；
- 变量之间相乘的非线性约束；
- 精确最大 Sharpe 等分式问题；
- 未经标量化的多目标优化。

多目标问题通常需要先转成加权目标，或者分阶段求解。整数规划需要使用 MILP/MIQP 求解器；
一般非线性问题则需要相应的非线性优化器。

一个简单的判断标准是：如果目标能写成凸二次函数，约束能写成线性等式或不等式，通常就可以
使用 Helix。

## 11. 从投资组合模块学习建模

项目中的投资组合实现可以作为新增业务优化器的参考：

- `src/helix/types.hpp`：通用问题、设置、状态和结果类型；
- `src/helix/osqp_solver.hpp`：通用 QP 求解器接口；
- `src/helix/lp_solver.hpp`：通用 LP 包装；
- `src/helix/portfolio.hpp`：领域输入、输出和 optimizer 接口；
- `src/helix/portfolio.cpp`：业务校验、约束构造和结果转换；
- `examples/portfolio/Portfolio_QP.cpp`：从均值—方差目标构造 QP 的完整示例。

新增优化任务时，通常不需要修改 `OsqpSolver`。优先新建一个领域建模模块，将业务输入转换为
`LpProblem` 或 `QpProblem`，调用求解器后再把 `SolveResult::x` 转换为业务结果。

## 12. Python 通用 QP API

Python 接口采用标准 QP 形式：

```text
minimize    0.5 * x.T @ P @ x + q.T @ x
subject to  l <= A @ x <= u
```

可以先构造问题对象，再复用同一个求解器：

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

print(result.x)
```

这里第一行约束表示 `x[0] + x[1] == 1`，后两行表示两个变量非负。变量上限、行业中性、
风险暴露等条件也需要按相同方式增加到 `A/l/u` 中。

修改 `problem.q`、`problem.l` 或 `problem.u` 后再次调用 `solver.solve(problem)`，在矩阵结构
没有变化时会复用 OSQP workspace。也可以使用 `solver.warm_start(x=..., y=...)` 显式设置
下一次求解的初始点。只求解一次时，可以直接调用：

```python
result = helix.solve_qp(P=P, q=q, A=A, l=l, u=u)
```

小型问题的 `P`、`A` 可以使用二维 NumPy 数组；大型问题应使用 SciPy CSC 或 CSR 稀疏矩阵。
完整可运行示例参见 `examples/python/qp.py`。
