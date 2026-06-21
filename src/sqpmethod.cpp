#include "ato_sim/sqpmethod.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ato {

namespace {
// 对向量做阻尼线搜索
double lineSearch(
    const std::vector<double>& x,
    const std::vector<double>& dx,
    const std::function<double(const std::vector<double>&)>& cost,
    double f0,
    double maxStep) {
    double step = 1.0;
    for (int it = 0; it < 12; ++it) {
        std::vector<double> xn(x.size());
        for (std::size_t i = 0; i < x.size(); ++i) {
            xn[i] = x[i] + std::min(step, maxStep) * dx[i];
        }
        const double fn = cost(xn);
        if (fn < f0 - 1e-12) {
            return std::min(step, maxStep);
        }
        step *= 0.5;
        if (step < 1e-6) {
            return step;
        }
    }
    return step;
}

// 数值梯度（中心差分）
void numericalGradient(
    const std::function<double(const std::vector<double>&)>& f,
    const std::vector<double>& x,
    std::vector<double>& grad,
    double h) {
    grad.assign(x.size(), 0.0);
    std::vector<double> xp = x;
    std::vector<double> xm = x;
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double xi = x[i];
        const double hi = std::max(h, 1e-7 * std::fabs(xi));
        xp[i] = xi + hi;
        xm[i] = xi - hi;
        const double fp = f(xp);
        const double fm = f(xm);
        grad[i] = (fp - fm) / (2.0 * hi);
        xp[i] = xi;
        xm[i] = xi;
    }
}

// 数值雅可比
void numericalJacobian(
    const std::function<void(std::size_t, const std::vector<double>&, double&)>& g,
    std::size_t nConstr,
    const std::vector<double>& x,
    mpc_internal::Matrix& J,
    double h) {
    const std::size_t n = x.size();
    J = mpc_internal::Matrix(nConstr, n);
    std::vector<double> xp = x;
    std::vector<double> xm = x;
    for (std::size_t j = 0; j < n; ++j) {
        const double xj = x[j];
        const double hj = std::max(h, 1e-7 * std::fabs(xj));
        xp[j] = xj + hj;
        xm[j] = xj - hj;
        for (std::size_t i = 0; i < nConstr; ++i) {
            double gp = 0.0, gm = 0.0;
            g(i, xp, gp);
            g(i, xm, gm);
            J(i, j) = (gp - gm) / (2.0 * hj);
        }
        xp[j] = xj;
        xm[j] = xj;
    }
}

// 投影到约束盒子：对每个约束 i，把 g(x) 限制到 [gLower, gUpper]。
// 这里不是直接投影 x，而是评估当前 x 的约束值并返回违反量。
// 我们用增广拉格朗日 / 罚函数法把约束违反量加到目标中。
struct ConstraintViolation {
    double totalViolation;       // 总违反量（L2 范数平方）
    std::vector<double> values;  // 每个约束的值
    std::vector<double> excess;  // 每个约束的违反量（超出上下界的部分）
};

ConstraintViolation evaluateConstraintViolation(
    std::size_t nConstr,
    const std::vector<double>& gLower,
    const std::vector<double>& gUpper,
    const std::function<void(std::size_t, const std::vector<double>&, double&)>& constraintValue,
    const std::vector<double>& x) {
    ConstraintViolation cv;
    cv.values.resize(nConstr, 0.0);
    cv.excess.resize(nConstr, 0.0);
    cv.totalViolation = 0.0;
    for (std::size_t i = 0; i < nConstr; ++i) {
        constraintValue(i, x, cv.values[i]);
        double excess = 0.0;
        if (cv.values[i] > gUpper[i]) {
            excess = cv.values[i] - gUpper[i];
        } else if (cv.values[i] < gLower[i]) {
            excess = cv.values[i] - gLower[i];  // 负值
        }
        cv.excess[i] = excess;
        cv.totalViolation += excess * excess;
    }
    return cv;
}

}  // namespace

Sqpmethod::Sqpmethod(SqpmethodConfig config) : config_(std::move(config)) {}

SqpmethodResult Sqpmethod::solve(
    std::size_t nVar,
    std::size_t nConstr,
    const std::vector<double>& x0,
    const std::vector<double>& gLower,
    const std::vector<double>& gUpper,
    std::function<double(const std::vector<double>&)> cost,
    std::function<void(const std::vector<double>&, std::vector<double>&)> gradient,
    std::function<void(std::size_t, const std::vector<double>&, double&)> constraintValue,
    std::function<void(const std::vector<double>&, mpc_internal::Matrix&)> constraintJacobian) {
    SqpmethodResult res;
    std::vector<double> x = x0;

    // 拉格朗日乘子初值（对偶变量）
    std::vector<double> lambda(nConstr, 0.0);

    // 罚参数（约束违反的二次惩罚系数）
    const double muInit = 10.0;
    const double muMax = 1e6;
    double mu = muInit;

    auto evaluateConstraints = [&](const std::vector<double>& xv) {
        std::vector<double> g(nConstr, 0.0);
        for (std::size_t i = 0; i < nConstr; ++i) {
            constraintValue(i, xv, g[i]);
        }
        return g;
    };

    // 增广目标 = 原始 cost + 约束罚
    auto augmentedCost = [&](const std::vector<double>& xv) -> double {
        double f = cost(xv);
        if (nConstr > 0) {
            auto cv = evaluateConstraintViolation(nConstr, gLower, gUpper,
                                                   constraintValue, xv);
            // 二次罚 + 一次拉格朗日项
            for (std::size_t i = 0; i < nConstr; ++i) {
                double excess = cv.excess[i];
                f += lambda[i] * excess + 0.5 * mu * excess * excess;
            }
        }
        return f;
    };

    for (int iter = 0; iter < config_.maxIterations; ++iter) {
        res.iterations = iter + 1;
        const double f = augmentedCost(x);
        res.cost = cost(x);  // 记录原始 cost（不含罚）

        std::vector<double> grad;
        if (gradient) {
            // 用原始梯度 + 约束梯度
            gradient(x, grad);
            if (nConstr > 0) {
                mpc_internal::Matrix J;
                if (constraintJacobian) {
                    constraintJacobian(x, J);
                } else {
                    numericalJacobian(constraintValue, nConstr, x, J, config_.finiteDiffStep);
                }
                auto cv = evaluateConstraintViolation(nConstr, gLower, gUpper,
                                                       constraintValue, x);
                for (std::size_t i = 0; i < nConstr; ++i) {
                    double excess = cv.excess[i];
                    // d(augmented)/dx = d(cost)/dx + sum_i [lambda_i + mu*excess_i] * d(g_i)/dx
                    double coeff = lambda[i] + mu * excess;
                    if (std::fabs(coeff) > 1e-15) {
                        for (std::size_t j = 0; j < nVar; ++j) {
                            grad[j] += coeff * J(i, j);
                        }
                    }
                }
            }
        } else {
            numericalGradient(augmentedCost, x, grad, config_.finiteDiffStep);
        }
        if (grad.size() != nVar) {
            throw std::runtime_error("gradient size mismatch");
        }

        // 收敛判定
        double gnorm = 0.0;
        for (double v : grad) gnorm = std::max(gnorm, std::fabs(v));
        res.gradientNorm = gnorm;

        // 约束违反量
        double cviol = 0.0;
        if (nConstr > 0) {
            auto cv = evaluateConstraintViolation(nConstr, gLower, gUpper,
                                                   constraintValue, x);
            for (double e : cv.excess) cviol = std::max(cviol, std::fabs(e));
        }

        if (gnorm < config_.gradientTol && cviol < 1e-4) {
            res.converged = true;
            res.x = x;
            res.cost = cost(x);
            return res;
        }

        // 搜索方向：-grad（梯度下降）
        std::vector<double> dx(nVar, 0.0);
        for (std::size_t i = 0; i < nVar; ++i) {
            dx[i] = -grad[i];
        }

        // 步长限幅
        for (std::size_t i = 0; i < nVar; ++i) {
            if (dx[i] > config_.maxStep) dx[i] = config_.maxStep;
            if (dx[i] < -config_.maxStep) dx[i] = -config_.maxStep;
        }

        // 线搜索（用增广目标）
        const double lsStep = lineSearch(x, dx, augmentedCost, f, config_.maxStep);
        if (lsStep < 1e-8) {
            // 步长太小，尝试增大罚参数并继续
            mu = std::min(mu * 4.0, muMax);
            if (mu >= muMax) {
                res.converged = false;
                res.x = x;
                res.cost = cost(x);
                return res;
            }
            continue;
        }

        std::vector<double> xNew(x.size());
        for (std::size_t i = 0; i < x.size(); ++i) {
            xNew[i] = x[i] + lsStep * dx[i];
        }

        // 收敛：||xNew - x|| < stepTol
        double dxn = 0.0;
        for (std::size_t i = 0; i < x.size(); ++i) {
            dxn = std::max(dxn, std::fabs(xNew[i] - x[i]));
        }
        if (dxn < config_.stepTol) {
            x = xNew;
            res.converged = true;
            res.x = x;
            res.cost = cost(x);
            return res;
        }
        x = xNew;

        // 更新拉格朗日乘子（对约束违反做一次估计）
        if (nConstr > 0) {
            auto cv = evaluateConstraintViolation(nConstr, gLower, gUpper,
                                                   constraintValue, x);
            for (std::size_t i = 0; i < nConstr; ++i) {
                lambda[i] += mu * cv.excess[i];
                // 限幅防止发散
                lambda[i] = std::max(-1e4, std::min(1e4, lambda[i]));
            }
            // 约束违反大时增大罚参数
            double maxExcess = 0.0;
            for (double e : cv.excess) maxExcess = std::max(maxExcess, std::fabs(e));
            if (maxExcess > 1e-3) {
                mu = std::min(mu * 1.5, muMax);
            }
        }
    }

    res.converged = false;
    res.x = x;
    res.cost = cost(x);
    return res;
}

}  // namespace ato