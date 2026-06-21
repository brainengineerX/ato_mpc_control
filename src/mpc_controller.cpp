#include "ato_sim/mpc_controller.hpp"

#include "ato_sim/linalg.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ato {

MpcController::MpcController(MpcConfig config)
    : config_(std::move(config)), plant_(config_.vehicle) {
    lastForce_.assign(config_.horizon, 0.0);
}

void MpcController::reset() {
    std::fill(lastForce_.begin(), lastForce_.end(), 0.0);
}

MpcPrediction MpcController::step(double currentSpeedCmps,
                                  double currentPositionCm,
                                  double currentGrade,
                                  const std::vector<ConstraintStep>& constraints,
                                  bool hasStop,
                                  double stopPositionCm,
                                  int horizon) {
    MpcPrediction pred;
    // 预测时域以调用方传入的 horizon 为准，但不超过配置上限。
    const int H = std::max(0, std::min(horizon, static_cast<int>(config_.horizon)));
    // 即便 H==0 也要满足头文件契约：speed/position 至少含初值(1 个)、force 为空。
    pred.speed.resize(static_cast<std::size_t>(H) + 1);
    pred.position.resize(static_cast<std::size_t>(H) + 1);
    pred.force.resize(static_cast<std::size_t>(H));
    pred.speed[0] = currentSpeedCmps;
    pred.position[0] = currentPositionCm;
    if (H <= 0) {
        pred.converged = true;
        pred.iterations = 0;
        return pred;
    }

    const auto t0 = std::chrono::steady_clock::now();

    const double T = config_.sampleTime;
    const double v0 = VehiclePlant::cmpsToMps(currentSpeedCmps);   // cm/s -> m/s
    const double s0 = VehiclePlant::cmToM(currentPositionCm);      // cm -> m
    const double sStop = VehiclePlant::cmToM(stopPositionCm);
    const double gradeRad = currentGrade * 1e-3;  // ‰ -> rad

    // 归一化决策变量 x ∈ [-1, 1] 对应实际力 u = x * Fscale（N）。
    const double Ftract = plant_.maxTractionForce(v0);
    const double Fbrake = plant_.maxBrakeForce(v0);
    const double Fscale = std::max(1.0, std::max(Ftract, Fbrake));

    const double mass = config_.vehicle.mass;
    const double effMass = mass * (1.0 + config_.vehicle.rotaryFactor);
    const double g = config_.vehicle.gravity;
    // 坡度阻力（N），上坡为正，需额外牵引克服。
    const double gradeForce = mass * g * std::sin(gradeRad);
    // Davis 阻力 R(v) = d0 + d1*v + d2*v^2，在当前速度 v0 处线性化：
    //   R(v) ≈ R0 + R0p*(v - v0)，R0p = d1 + 2*d2*v0。
    const double d0 = config_.vehicle.davisA0;
    const double d1 = config_.vehicle.davisA1;
    const double d2 = config_.vehicle.davisA2;
    const double R0 = d0 + d1 * v0 + d2 * v0 * v0;
    const double R0p = d1 + 2.0 * d2 * v0;

    // 线性化动力学（关于 x 仿射）：v_{k+1} = v_k + T*a_k，
    // a_k = (x_k*Fscale - R0 - R0p*(v_k - v0) - gradeForce)/effMass。
    // 展开见下方 aConstK / axKj。

    // 参考速度/最大速度采样（m/s）。
    auto vrefAt = [&](int k) -> double {
        if (constraints.empty()) return 0.0;
        const int idx = std::min(k, static_cast<int>(constraints.size()) - 1);
        return VehiclePlant::cmpsToMps(constraints[static_cast<std::size_t>(idx)].targetSpeed);
    };
    auto maxSpeedAt = [&](int k) -> double {
        if (constraints.empty()) return std::numeric_limits<double>::infinity();
        const int idx = std::min(k, static_cast<int>(constraints.size()) - 1);
        return VehiclePlant::cmpsToMps(constraints[static_cast<std::size_t>(idx)].maxSpeed);
    };

    // 前向展开 v_k、s_k 为 x 的仿射函数：
    //   v_k = a_v[k] + Σ_j B_v[k][j]*x_j
    //   s_k = a_s[k] + Σ_j B_s[k][j]*x_j
    std::vector<double> aV(H + 1), aS(H + 1);
    std::vector<std::vector<double>> BV(H + 1, std::vector<double>(H, 0.0));
    std::vector<std::vector<double>> BS(H + 1, std::vector<double>(H, 0.0));
    aV[0] = v0;
    aS[0] = s0;
    for (int k = 0; k < H; ++k) {
        // a_k = (x_k*Fscale - R0 - R0p*(v_k - v0) - gradeForce)/effMass
        // 常数部分 a_const_k = (-R0 - R0p*(aV[k]-v0) - gradeForce)/effMass
        // x 部分 a_x[k][j] = (Fscale*δ_{j,k} - R0p*BV[k][j])/effMass
        const double aConstK = (-R0 - R0p * (aV[k] - v0) - gradeForce) / effMass;
        // v_{k+1} = v_k + T*a_k
        aV[k + 1] = aV[k] + T * aConstK;
        for (int j = 0; j < H; ++j) {
            const double axKj = ((j == k ? Fscale : 0.0) - R0p * BV[k][j]) / effMass;
            BV[k + 1][j] = BV[k][j] + T * axKj;
        }
        // s_{k+1} = s_k + T*v_k + 0.5*T^2*a_k
        aS[k + 1] = aS[k] + T * aV[k] + 0.5 * T * T * aConstK;
        for (int j = 0; j < H; ++j) {
            const double axKj = ((j == k ? Fscale : 0.0) - R0p * BV[k][j]) / effMass;
            BS[k + 1][j] = BS[k][j] + T * BV[k][j] + 0.5 * T * T * axKj;
        }
    }

    // 组装无约束 QP：min 0.5*x'R x + q'x。
    // 代价项（每项 c*(d + B·x)^2 贡献 R += 2c*B*B', q += 2c*d*B）。
    const std::size_t n = static_cast<std::size_t>(H);
    mpc_internal::Matrix R(n, n, 0.0);
    std::vector<double> q(n, 0.0);

    auto addTerm = [&](const std::vector<double>& Bk, double d, double c) {
        for (std::size_t i = 0; i < n; ++i) {
            q[i] += 2.0 * c * d * Bk[i];
            for (std::size_t j = 0; j < n; ++j) {
                R(i, j) += 2.0 * c * Bk[i] * Bk[j];
            }
        }
    };

    // 速度跟踪：k=1..H（k=0 项与 x 无关，略）。
    for (int k = 1; k <= H; ++k) {
        const double d = aV[k] - vrefAt(k);
        addTerm(BV[k], d, config_.wSpeed);
    }
    // 软超速约束：当预测速度超过该步 maxSpeed（EBI/限速走廊）时，对超出量
    // 施加二次惩罚，把速度压回安全包络。这是 active-set 风格近似——仅在当前
    // 展开点 aV[k] 已超 maxSpeed 的步上激活，避免无约束 QP 表达分段函数。
    // 对应 header 中 wSlackSpeed（越界速度）与 maxSpeedAt（此前未使用）。
    // 注意：跳过 maxSpeed<=0 的步——那是停车点本身，已由 wStopSpeed 末端
    // 速度项与 wTerminal 位置项处理，此处再用大权重会主导并扰乱停车解。
    for (int k = 1; k <= H; ++k) {
        const double vmax = maxSpeedAt(k);
        if (std::isfinite(vmax) && vmax > 0.0 && aV[k] > vmax) {
            addTerm(BV[k], aV[k] - vmax, config_.wSlackSpeed);
        }
    }
    // 控制能量：wControl*x_k^2。
    for (std::size_t k = 0; k < n; ++k) {
        R(k, k) += 2.0 * config_.wControl;
    }
    // 冲击平滑：wJerk*(x_k - x_{k-1})^2。x_{-1} = lastForce_[0]（热启动）。
    const double xPrev0 = lastForce_.empty() ? 0.0 : lastForce_[0];
    {
        std::vector<double> e0(n, 0.0);
        e0[0] = 1.0;
        addTerm(e0, -xPrev0, config_.wJerk);  // (x_0 - xPrev)^2
    }
    for (std::size_t k = 1; k < n; ++k) {
        R(k, k) += 2.0 * config_.wJerk;
        R(k, k - 1) -= 2.0 * config_.wJerk;
        R(k - 1, k) -= 2.0 * config_.wJerk;
    }
    // 停车段：末端速度归零 + 末端位置精度。
    if (hasStop) {
        addTerm(BV[H], aV[H], config_.wStopSpeed);  // wStopSpeed*v_H^2
        // 单侧位置惩罚：仅当"不施加控制也会到达/越过停车点"(aS[H]>=sStop)时
        // 施加，防止远处 s_H<sStop 把解推向加速去够停车点（导致过冲）。
        if (aS[H] >= sStop) {
            const double d = aS[H] - sStop;
            addTerm(BS[H], d, config_.wTerminal);
        }
    }

    // 求解 R*x = -q（R 对称正定：wControl>0 保证）。
    std::vector<double> xOpt(n, 0.0);
    bool converged = false;
    // iterations 语义：直接线性求解无"迭代"概念，用 1 表示本周期成功求解、
    // 0 表示未求解（RTI 热启动）或求解失败回退到上一周期解。这样消费者可据
    // 此区分"新鲜解"与"回退解"，而非恒为 1。
    int iterations = 0;
    if (config_.enableSolver) {
        std::vector<double> rhs(n);
        for (std::size_t i = 0; i < n; ++i) rhs[i] = -q[i];
        converged = R.solveSymmetric(rhs, xOpt);
        if (converged) {
            iterations = 1;
        } else {
            // 退化（R 非正定，极少发生）：回退到上一周期解。
            for (std::size_t i = 0; i < n && i < lastForce_.size(); ++i) {
                xOpt[i] = lastForce_[i];
            }
        }
    } else {
        // RTI 风格：不求解，直接用上一周期解。
        for (std::size_t i = 0; i < n && i < lastForce_.size(); ++i) {
            xOpt[i] = lastForce_[i];
        }
    }

    // 归一化域限幅。
    for (std::size_t i = 0; i < n; ++i) {
        xOpt[i] = std::max(-1.0, std::min(1.0, xOpt[i]));
    }

    // ---- B1/B2 速度硬约束 ----
    // v_k = aV[k] + BV[k]·x 是 x 的仿射函数。约束 v_k <= maxSpeedAt(k) (m/s)。
    // 跳过 maxSpeed<=0 的步（停车点，由末端项处理）。
    if (config_.constraintMode == MpcConstraintMode::Projected) {
        // B1 投影法：迭代检查预测速度，对超限步按灵敏度把控制量往制动方向压。
        auto predictV = [&](int k) -> double {
            double v = aV[k];
            for (std::size_t j = 0; j < n; ++j) v += BV[k][j] * xOpt[j];
            return v;
        };
        for (int it = 0; it < config_.constraintMaxIter; ++it) {
            bool anyOver = false;
            for (int k = 1; k <= H; ++k) {
                const double vmax = maxSpeedAt(k);
                if (!(std::isfinite(vmax) && vmax > 0.0)) continue;
                const double vPred = predictV(k);
                const double over = vPred - vmax;
                if (over > 1e-4) {
                    anyOver = true;
                    // x_{k-1} 对 v_k 的灵敏度 = BV[k][k-1]。
                    // 减少 x_{k-1} 使 v_k 降 over：Δx = over / BV[k][k-1]。
                    const std::size_t idx = static_cast<std::size_t>(k - 1);
                    if (idx < n) {
                        const double sens = BV[k][idx];
                        if (std::fabs(sens) > 1e-9) {
                            xOpt[idx] -= over / sens;
                            xOpt[idx] = std::max(-1.0, std::min(1.0, xOpt[idx]));
                        } else {
                            // 灵敏度太小，直接施加制动
                            xOpt[idx] = std::max(-1.0, xOpt[idx] - 0.1);
                        }
                    }
                }
            }
            if (!anyOver) break;
        }
    } else if (config_.constraintMode == MpcConstraintMode::ActiveSet) {
        // B2 主动集：把 v_k<=vmax 线性化为 BV[k]·x <= vmax_k - aV[k]。
        // 迭代：解带主动约束的等式 QP（KKT），找最大违反者加入主动集，直到满足。
        std::vector<int> activeSet;  // 主动约束的步索引 k
        for (int it = 0; it < config_.constraintMaxIter; ++it) {
            const std::size_t na = activeSet.size();
            if (na > 0) {
                // 解 KKT: [R A'; A 0][x;λ] = [-q; b]，A 行 = BV[k]，b = vmax_k - aV[k]。
                mpc_internal::Matrix K(n + na, n + na, 0.0);
                for (std::size_t i = 0; i < n; ++i)
                    for (std::size_t j = 0; j < n; ++j) K(i, j) = R(i, j);
                std::vector<double> rhs(n + na, 0.0);
                for (std::size_t i = 0; i < n; ++i) rhs[i] = -q[i];
                for (std::size_t a = 0; a < na; ++a) {
                    const int k = activeSet[a];
                    const std::size_t row = n + a;
                    for (std::size_t j = 0; j < n; ++j) {
                        K(row, j) = BV[k][j];
                        K(j, row) = BV[k][j];
                    }
                    rhs[row] = maxSpeedAt(k) - aV[k];
                }
                std::vector<double> sol(n + na, 0.0);
                if (K.solveGeneral(rhs, sol)) {
                    for (std::size_t i = 0; i < n; ++i) xOpt[i] = sol[i];
                    for (std::size_t i = 0; i < n; ++i) {
                        xOpt[i] = std::max(-1.0, std::min(1.0, xOpt[i]));
                    }
                }
            }
            // 找最大违反者
            int worstK = -1;
            double worstOver = 1e-6;  // 容差
            for (int k = 1; k <= H; ++k) {
                const double vmax = maxSpeedAt(k);
                if (!(std::isfinite(vmax) && vmax > 0.0)) continue;
                double vPred = aV[k];
                for (std::size_t j = 0; j < n; ++j) vPred += BV[k][j] * xOpt[j];
                const double over = vPred - vmax;
                bool inSet = false;
                for (int ak : activeSet) if (ak == k) { inSet = true; break; }
                if (!inSet && over > worstOver) { worstOver = over; worstK = k; }
            }
            if (worstK < 0) break;  // 所有约束满足
            activeSet.push_back(worstK);
        }
    }

    // 用真实非线性 plant 重新预测输出轨迹（保证与仿真层 plant 行为一致）。
    std::vector<double> vSeq(H + 1, 0.0), sSeq(H + 1, 0.0);
    vSeq[0] = v0;
    sSeq[0] = s0;
    VehicleState vs;
    vs.grade = gradeRad;
    for (int k = 0; k < H; ++k) {
        vs.position = sSeq[k];
        vs.speed = vSeq[k];
        const double u = xOpt[static_cast<std::size_t>(k)] * Fscale;
        const auto r = plant_.step(vs, u);
        sSeq[k + 1] = r.state.position;
        vSeq[k + 1] = r.state.speed;
    }

    pred.speed.resize(n + 1);
    pred.position.resize(n + 1);
    pred.force.resize(n);
    for (int k = 0; k <= H; ++k) {
        pred.speed[static_cast<std::size_t>(k)] = VehiclePlant::mpsToCmps(vSeq[k]);
        pred.position[static_cast<std::size_t>(k)] = VehiclePlant::mToCm(sSeq[k]);
    }
    for (int k = 0; k < H; ++k) {
        double u = xOpt[static_cast<std::size_t>(k)] * Fscale;  // N
        u = std::max(-Fbrake, std::min(Ftract, u));
        pred.force[static_cast<std::size_t>(k)] = u;
    }

    // 代价（用真实轨迹重算，供观测）。
    {
        double J = 0.0;
        double prevX = xPrev0;
        for (int k = 0; k < H; ++k) {
            const double dv = vSeq[k] - vrefAt(k);
            J += config_.wSpeed * dv * dv;
            J += config_.wControl * xOpt[k] * xOpt[k];
            const double dx = xOpt[k] - prevX;
            J += config_.wJerk * dx * dx;
            prevX = xOpt[k];
        }
        const double dvH = vSeq[H] - vrefAt(H);
        J += config_.wSpeed * dvH * dvH;
        if (hasStop) {
            J += config_.wStopSpeed * vSeq[H] * vSeq[H];
            if (aS[H] >= sStop) {
                const double ds = sSeq[H] - sStop;
                J += config_.wTerminal * ds * ds;
            }
        }
        pred.cost = J;
    }
    pred.converged = converged;
    pred.iterations = iterations;
    const auto t1 = std::chrono::steady_clock::now();
    pred.solveTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 缓存本周期解作为下一周期热启动初值。
    lastForce_.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        lastForce_[i] = xOpt[i];
    }

    return pred;
}

}  // namespace ato
