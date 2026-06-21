#include "ato_sim/evaluator.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ato {

EvaluationMetrics Evaluator::evaluate(
    const std::string& scenario,
    const std::vector<TrajectoryPoint>& traj,
    double stopPointCm,
    double expectedStopTimeS,
    double sampleTimeS) {
    EvaluationMetrics m;
    m.scenario = scenario;
    m.totalSteps = static_cast<int>(traj.size());
    if (traj.empty()) {
        return m;
    }

    double sumSqErr = 0.0;
    int speedCount = 0;
    double maxErr = 0.0;
    double sumJerkSq = 0.0;
    double maxJerk = 0.0;
    double sumSolve = 0.0;
    double maxSolve = 0.0;
    int solveCount = 0;
    int lastDir = traj.front().direction;
    // 加速度统计
    double sumAccelSq = 0.0;
    double maxAccel = 0.0;
    int accelCount = 0;
    // 超速统计
    int overEbiSteps = 0;
    double overEbiMax = 0.0;
    int overSbiSteps = 0;
    double overSbiMax = 0.0;
    // 能耗分项
    double tractionEnergy = 0.0;
    double brakeEnergy = 0.0;

    for (std::size_t i = 0; i < traj.size(); ++i) {
        const TrajectoryPoint& p = traj[i];
        // 速度跟踪误差（仅在目标速度 > 0 时统计）
        if (p.targetSpeed > 1.0) {
            const double err = p.trainSpeed - p.targetSpeed;
            sumSqErr += err * err;
            speedCount++;
            maxErr = std::max(maxErr, std::fabs(err));
        }
        // Jerk
        if (i > 0) {
            const double da = p.acceleration - traj[i - 1].acceleration;
            const double j = da / sampleTimeS;
            sumJerkSq += j * j;
            maxJerk = std::max(maxJerk, std::fabs(j));
        }
        // 加速度 RMS / 最大
        if (p.targetSpeed > 1.0 || p.trainSpeed > 1.0) {
            sumAccelSq += p.acceleration * p.acceleration;
            maxAccel = std::max(maxAccel, std::fabs(p.acceleration));
            accelCount++;
        }
        // 方向切换
        if (i > 0 && p.direction != lastDir) {
            if ((lastDir == 0 && p.direction != 0) ||
                (lastDir != 0 && p.direction == 0) ||
                (lastDir == 1 && p.direction == 2) ||
                (lastDir == 2 && p.direction == 1)) {
                m.directionSwitches++;
            }
            // 牵引<->制动 直接切换（1<->2，含跨 coast 的 1->0->2 不计）。
            if ((lastDir == 1 && p.direction == 2) ||
                (lastDir == 2 && p.direction == 1)) {
                m.tractionBrakeSwitches++;
            }
        }
        lastDir = p.direction;

        // 求解器耗时（仅 MPC 路径有非零值）
        if (p.solveTimeMs > 0.0) {
            sumSolve += p.solveTimeMs;
            maxSolve = std::max(maxSolve, p.solveTimeMs);
            solveCount++;
        }

        // 超速：列车速度越过 EBI / SBI(最高推荐速度) 包络。
        if (p.ebiSpeed > 0.0 && p.trainSpeed > p.ebiSpeed) {
            overEbiSteps++;
            overEbiMax = std::max(overEbiMax, p.trainSpeed - p.ebiSpeed);
        }
        if (p.maxRecommendedSpeed > 0.0 && p.trainSpeed > p.maxRecommendedSpeed) {
            overSbiSteps++;
            overSbiMax = std::max(overSbiMax, p.trainSpeed - p.maxRecommendedSpeed);
        }
    }

    m.avgSolveTimeMs = (solveCount > 0) ? sumSolve / solveCount : 0.0;
    m.maxSolveTimeMs = maxSolve;

    m.speedRmse = (speedCount > 0) ? std::sqrt(sumSqErr / speedCount) : 0.0;
    m.speedMaxError = maxErr;
    m.jerkRms = (traj.size() > 1) ? std::sqrt(sumJerkSq / (traj.size() - 1)) : 0.0;
    m.jerkMax = maxJerk;
    m.accelRms = (accelCount > 0) ? std::sqrt(sumAccelSq / accelCount) : 0.0;
    m.accelMax = maxAccel;
    m.overspeedEbiSteps = overEbiSteps;
    m.overspeedEbiMaxCmps = overEbiMax;
    m.overspeedSbiSteps = overSbiSteps;
    m.overspeedSbiMaxCmps = overSbiMax;

    // 能耗：∫|F|*v dt。正向力(牵引)与负向力(制动)分开累计，
    // 后者可视为再生制动可回收的能量上限。
    double energy = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
        const double v = traj[i].trainSpeed * 0.01;  // m/s
        const double f = traj[i].force;
        const double dt = sampleTimeS;
        const double work = f * v * dt;  // J，牵引为正、制动为负
        energy += std::fabs(work);
        if (work > 0.0) {
            tractionEnergy += work;
        } else {
            brakeEnergy += -work;
        }
    }
    m.energy = energy;
    m.tractionEnergy = tractionEnergy;
    m.brakeEnergy = brakeEnergy;

    // 运行时间：首帧到末帧。
    m.runTimeS = traj.back().time - traj.front().time;

    // 停车误差
    if (stopPointCm > 0.0 && !traj.empty()) {
        const double finalPos = traj.back().position;
        m.stopError = std::fabs(finalPos - stopPointCm);
        if (expectedStopTimeS > 0.0) {
            m.stopTimeError = traj.back().time - expectedStopTimeS;
        }
    }

    return m;
}

std::string Evaluator::toCsvRow(const EvaluationMetrics& m) {
    std::ostringstream oss;
    oss << m.scenario << ","
        << std::fixed << std::setprecision(3)
        << m.stopError << ","
        << m.stopTimeError << ","
        << m.speedRmse << ","
        << m.speedMaxError << ","
        << m.jerkRms << ","
        << m.jerkMax << ","
        << m.directionSwitches << ","
        << m.energy << ","
        << m.avgSolveTimeMs << ","
        << m.maxSolveTimeMs << ","
        << m.totalSteps << ","
        << m.runTimeS << ","
        << m.tractionEnergy << ","
        << m.brakeEnergy << ","
        << m.overspeedEbiSteps << ","
        << m.overspeedEbiMaxCmps << ","
        << m.overspeedSbiSteps << ","
        << m.overspeedSbiMaxCmps << ","
        << m.accelRms << ","
        << m.accelMax << ","
        << m.tractionBrakeSwitches;
    return oss.str();
}

static const char* kCsvHeader =
    "scenario,stop_error_cm,stop_time_err_s,speed_rmse_cmps,speed_max_err_cmps,"
    "jerk_rms_cmps3,jerk_max_cmps3,dir_switches,energy_j,avg_solve_ms,max_solve_ms,steps,"
    "run_time_s,traction_energy_j,brake_energy_j,"
    "over_ebi_steps,over_ebi_max_cmps,over_sbi_steps,over_sbi_max_cmps,"
    "accel_rms_cmps2,accel_max_cmps2,trac_brake_switches";

std::string Evaluator::toMarkdownTable(
    const std::vector<EvaluationMetrics>& pidMetrics,
    const std::vector<EvaluationMetrics>& mpcMetrics) {
    std::ostringstream oss;
    oss << "# PID vs MPC 仿真评估对比\n\n";
    oss << "## 场景对比\n\n";
    oss << "| 场景 | 停车误差 PID (cm) | 停车误差 MPC (cm) | "
        << "速度RMSE PID (cm/s) | 速度RMSE MPC (cm/s) | "
        << "Jerk RMS PID (cm/s³) | Jerk RMS MPC (cm/s³) | "
        << "加速度RMS PID (cm/s²) | 加速度RMS MPC (cm/s²) | "
        << "切换 PID | 切换 MPC | 牵制切换 PID | 牵制切换 MPC | "
        << "总能耗 PID (kJ) | 总能耗 MPC (kJ) | "
        << "再生上限 PID (kJ) | 再生上限 MPC (kJ) | "
        << "运行时间 PID (s) | 运行时间 MPC (s) | "
        << "超EBI步 PID | 超EBI步 MPC | 超SBI步 PID | 超SBI步 MPC |\n";
    oss << "|------|------------|------------|------------|------------|"
        << "------------|------------|------------|------------|"
        << "------|------|--------|--------|--------|--------|--------|--------|"
        << "--------|--------|--------|--------|--------|\n";

    for (std::size_t i = 0; i < pidMetrics.size() && i < mpcMetrics.size(); ++i) {
        const auto& p = pidMetrics[i];
        const auto& m = mpcMetrics[i];
        oss << "| " << p.scenario
            << " | " << std::fixed << std::setprecision(1) << p.stopError
            << " | " << m.stopError
            << " | " << std::setprecision(2) << p.speedRmse
            << " | " << m.speedRmse
            << " | " << p.jerkRms
            << " | " << m.jerkRms
            << " | " << p.accelRms
            << " | " << m.accelRms
            << " | " << p.directionSwitches
            << " | " << m.directionSwitches
            << " | " << p.tractionBrakeSwitches
            << " | " << m.tractionBrakeSwitches
            << " | " << std::setprecision(1) << (p.energy / 1000.0)
            << " | " << (m.energy / 1000.0)
            << " | " << (p.brakeEnergy / 1000.0)
            << " | " << (m.brakeEnergy / 1000.0)
            << " | " << std::setprecision(2) << p.runTimeS
            << " | " << m.runTimeS
            << " | " << p.overspeedEbiSteps
            << " | " << m.overspeedEbiSteps
            << " | " << p.overspeedSbiSteps
            << " | " << m.overspeedSbiSteps << " |\n";
    }
    oss << "\n## 总结\n\n";
    oss << "- 停车精度、速度跟踪、Jerk、加速度反映控制质量；\n"
        << "- 总能耗 / 再生上限(制动能量) / 运行时间反映运行经济性与效率；\n"
        << "- 超 EBI / 超 SBI 步数反映安全裕度（应为 0 或极少）。\n"
        << "MPC 求解器配置、参数细节见 mpc_design.md。\n";
    return oss.str();
}

bool Evaluator::writeCsv(const std::string& path,
                         const std::vector<EvaluationMetrics>& metrics) {
    std::ofstream of(path);
    if (!of) {
        return false;
    }
    of << kCsvHeader << "\n";
    for (const auto& m : metrics) {
        of << toCsvRow(m) << "\n";
    }
    return of.good();
}

}  // namespace ato
