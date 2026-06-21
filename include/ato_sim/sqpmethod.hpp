#pragma once

#include "ato_sim/linalg.hpp"
#include "ato_sim/vehicle_plant.hpp"

#include <vector>

namespace ato {

// 单个 SQP 子问题设置。
struct SqpmethodConfig {
    // 最大 SQP 外层迭代次数
    int maxIterations = 8;
    // 收敛容差：||∇L||_∞
    double gradientTol = 1e-4;
    // 收敛容差：||Δu||
    double stepTol = 1e-5;
    // 一维搜索步长（阻尼）上限
    double maxStep = 1.0;
    // 数值微分步长
    double finiteDiffStep = 1e-5;
};

// SQP 求解结果
struct SqpmethodResult {
    bool converged = false;
    int iterations = 0;
    double cost = 0.0;
    double gradientNorm = 0.0;
    std::vector<double> x;  // 最优解向量
};

// 简单 dense SQP（Sequential Quadratic Programming）求解器。
// 目标是最小化 f(x) 满足 g(x) <= 0（向量约束）。
// 用于 MPC 内部：决策变量是未来 H 步的牵引/制动力。
//
// 不依赖任何外部库；使用中心差分估计梯度，LDL^T 求解 QP 子问题，
// 用简单回溯线搜索保证下降。
class Sqpmethod {
public:
    explicit Sqpmethod(SqpmethodConfig config);

    // 求解最优化问题：
    //   min   f(x)
    //   s.t.  g_lower <= g(x) <= g_upper
    //
    // callbacks:
    //   cost(x) -> 标量目标
    //   gradient(x, g) -> 填充 ∇f
    //   constraintValue(i, x) -> 第 i 个约束当前值
    //   constraintJacobian(x, J) -> J(i,j) = ∂g_i/∂x_j，密集矩阵 (n_constr, n_var)
    //   applyHessian(...) 可选，默认用 BFGS 近似
    SqpmethodResult solve(
        std::size_t nVar,
        std::size_t nConstr,
        const std::vector<double>& x0,
        const std::vector<double>& gLower,
        const std::vector<double>& gUpper,
        std::function<double(const std::vector<double>&)> cost,
        std::function<void(const std::vector<double>&, std::vector<double>&)> gradient,
        std::function<void(std::size_t, const std::vector<double>&, double&)> constraintValue,
        std::function<void(const std::vector<double>&, mpc_internal::Matrix&)> constraintJacobian);

private:
    SqpmethodConfig config_;
};

}  // namespace ato
