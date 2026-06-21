#include "ato_sim/control_loop.hpp"
#include "ato_sim/evaluator.hpp"
#include "ato_sim/vehicle_plant.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

// 轻量 CSV 解析器：当前仿真输入不支持带引号逗号，只用于数值场景文件。
std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(cell);
    }
    return cells;
}

double numberAt(const std::vector<std::string>& cells, std::size_t index, double fallback = 0.0) {
    if (index >= cells.size() || cells[index].empty()) {
        return fallback;
    }
    return std::strtod(cells[index].c_str(), nullptr);
}

int intAt(const std::vector<std::string>& cells, std::size_t index, int fallback = 0) {
    return static_cast<int>(numberAt(cells, index, static_cast<double>(fallback)));
}

bool boolAt(const std::vector<std::string>& cells, std::size_t index, bool fallback = false) {
    return intAt(cells, index, fallback ? 1 : 0) != 0;
}

ato::DrivePhase phaseAt(const std::vector<std::string>& cells, std::size_t index) {
    if (index >= cells.size()) {
        return ato::DrivePhase::Cruise;
    }
    return (cells[index] == "stop" || cells[index] == "STOP" || cells[index] == "1")
               ? ato::DrivePhase::Stop
               : ato::DrivePhase::Cruise;
}

// 解析一对 distance;distance;...;distance / speed;... / grade;... 形式的限速列表。
std::vector<ato::SpeedRestriction> parseRestrictions(
    const std::vector<std::string>& cells, std::size_t idxDist,
    std::size_t idxSpeed, std::size_t idxGrade, double defaultGrade) {
    std::vector<ato::SpeedRestriction> out;
    if (idxDist >= cells.size()) {
        return out;
    }
    auto splitMulti = [](const std::string& s) {
        std::vector<std::string> parts;
        std::stringstream ss(s);
        std::string p;
        while (std::getline(ss, p, ';')) {
            parts.push_back(p);
        }
        return parts;
    };
    const auto dists = splitMulti(cells[idxDist]);
    const auto speeds = (idxSpeed < cells.size()) ? splitMulti(cells[idxSpeed]) : std::vector<std::string>{};
    const auto grades = (idxGrade < cells.size()) ? splitMulti(cells[idxGrade]) : std::vector<std::string>{};
    for (std::size_t i = 0; i < dists.size(); ++i) {
        if (dists[i].empty()) continue;
        const double d = std::strtod(dists[i].c_str(), nullptr);
        const double v = (i < speeds.size() && !speeds[i].empty())
            ? std::strtod(speeds[i].c_str(), nullptr) : 0.0;
        const double g = (i < grades.size() && !grades[i].empty())
            ? std::strtod(grades[i].c_str(), nullptr) : defaultGrade;
        if (d >= 0.0 && v > 0.0) {
            out.push_back({d, v, g});
        }
    }
    return out;
}

// 一行场景的解析结果。time_s 为该行时间戳，其余为该时刻的线路/目标输入。
struct ScenarioRow {
    double time = 0.0;
    double ebiSpeed = 2200.0;
    double runLevelSpeed = 2200.0;
    double distanceToStop = ato::kInvalidDistance;
    double grade = 0.0;
    ato::DrivePhase phase = ato::DrivePhase::Cruise;
    bool trainStopped = false;
    int stopPointId = 0;
    std::vector<ato::SpeedRestriction> restrictions;
};

ato::ControlConfig defaultConfig() {
    ato::ControlConfig cfg;

    cfg.parking.brakeAccParaA = 0.0;
    cfg.parking.brakeAccParaB = 100.0;
    cfg.parking.originBrakeAcc = 100.0;
    cfg.parking.defaultBrakeAcc = 100.0;

    cfg.target.maxLineLimitSpeed = 2200.0;
    cfg.target.defaultBrakeAcc = 100.0;
    cfg.target.minMarginFromEbi = 55.0;

    cfg.ebSb.motorAcc = 80.0;
    cfg.ebSb.brakeAcc = 120.0;
    cfg.ebSb.motorDelay = 1.0;
    cfg.ebSb.brakeDelay = 2.0;

    // 车辆牵引/制动能力曲线（output 与 mpc 共用）。
    ato::Curve tracCurve = {{0.0, 100.0}, {600.0, 100.0}, {1600.0, 75.0}, {2400.0, 45.0}};
    ato::Curve brakeCurve = {{0.0, 110.0}, {600.0, 110.0}, {1600.0, 105.0}, {2400.0, 95.0}};

    cfg.output.tractionCurve = tracCurve;
    cfg.output.brakeCurve = brakeCurve;
    cfg.output.minAnalogOutput = 1000.0;
    cfg.output.maxAnalogOutput = 9000.0;
    cfg.output.tractionRiseStep = 500.0;
    cfg.output.tractionFallStep = 500.0;
    cfg.output.brakeRiseStep = 500.0;
    cfg.output.brakeFallStep = 500.0;
    cfg.output.brakeHoldSpeed = 5.0;
    cfg.output.brakeHoldPercent = 20.0;

    cfg.autoAdjust.enabled = true;
    cfg.autoAdjust.maxError = 40;
    cfg.autoAdjust.defaultAdjustValue = 0;
    cfg.autoAdjust.abnormalMaxRecord = 3;

    // MPC 配置
    cfg.mpc.horizon = 20;
    cfg.mpc.sampleTime = 0.2;
    cfg.mpc.wSpeed = 5.0;         // 速度跟踪（紧跟踪参考减速曲线）
    cfg.mpc.wControl = 1e-2;      // 控制量权重，抑制 bang-bang
    cfg.mpc.wJerk = 5e-2;         // 冲击权重，平滑相邻控制
    cfg.mpc.wTerminal = 2000.0;   // 末端位置精度（强约束停车点）
    cfg.mpc.wStopSpeed = 500.0;   // 末端速度归零
    cfg.mpc.vehicle.mass = 200000.0;
    cfg.mpc.vehicle.rotaryFactor = 0.10;
    cfg.mpc.vehicle.davisA0 = 2000.0;
    cfg.mpc.vehicle.davisA1 = 30.0;
    cfg.mpc.vehicle.davisA2 = 3.0;
    cfg.mpc.vehicle.tractionCurve = tracCurve;
    cfg.mpc.vehicle.brakeCurve = brakeCurve;
    cfg.mpc.vehicle.sampleTime = 0.2;
    cfg.mpc.solver.maxIterations = 25;
    cfg.mpc.solver.gradientTol = 1e-3;
    cfg.mpc.solver.stepTol = 1e-4;
    return cfg;
}

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [--evaluate] "
        << "[--duration=SECONDS] <scenario.csv>\n\n"
        << "CSV columns:\n"
        << "  0 time_s\n"
        << "  1 train_speed (initial speed at t0; thereafter plant-driven)\n"
        << "  2 ebi_speed\n"
        << "  3 run_level_speed\n"
        << "  4 distance_to_stop\n"
        << "  5 grade (per-mille)\n"
        << "  6 phase (cruise|stop|0|1)\n"
        << "  7 train_stopped\n"
        << "  8 stop_point_id\n"
        << "  9+ restriction_distance;...  restriction_speed;...  restriction_grade;...\n"
        << "\n"
        << "Multi-restriction: use ';' separator inside the cell, e.g. '5000;3000;1500'.\n"
        << "Closed-loop: train speed is integrated by the vehicle plant from the\n"
        << "controller output; the CSV speed column only seeds the initial state.\n"
        << "--duration overrides cruise-scenario end time so the train can reach\n"
        << "cruise speed (cruise scenarios otherwise stop at the CSV's last time).\n"
        << "If --evaluate is set, prints EvaluationMetrics after the trajectory.\n";
}

// 在场景时间轴上采样时刻 t 的线路/目标输入。
// 连续量(ebi/runLevel/grade)线性插值；离散量(phase/stopPointId/限制/停车标志)
// 用不晚于 t 的最后一行(step-hold)。
ScenarioRow sampleAt(const std::vector<ScenarioRow>& rows, double t) {
    ScenarioRow s;
    if (rows.empty()) return s;
    if (t <= rows.front().time) return rows.front();
    if (t >= rows.back().time) return rows.back();

    std::size_t i = 0;
    while (i + 1 < rows.size() && rows[i + 1].time <= t) ++i;
    const ScenarioRow& a = rows[i];
    const ScenarioRow& b = rows[i + 1];
    const double span = b.time - a.time;
    const double ratio = (span > 0.0) ? (t - a.time) / span : 0.0;
    s.time = t;
    s.ebiSpeed = a.ebiSpeed + ratio * (b.ebiSpeed - a.ebiSpeed);
    s.runLevelSpeed = a.runLevelSpeed + ratio * (b.runLevelSpeed - a.runLevelSpeed);
    s.grade = a.grade + ratio * (b.grade - a.grade);
    // 离散量 step-hold：用不晚于 t 的最后一行
    s.distanceToStop = a.distanceToStop;
    s.phase = a.phase;
    s.trainStopped = a.trainStopped;
    s.stopPointId = a.stopPointId;
    s.restrictions = a.restrictions;
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    bool evaluate = false;
    double duration = -1.0;  // <0 表示不指定，按场景决定
    ato::MpcConstraintMode cmode = ato::MpcConstraintMode::Soft;
    double vrefMargin = -1.0;  // <0 用默认
    double vrefMarginFar = -1.0;  // <0 用默认(禁用分段)
    double vrefFarDist = -1.0;  // <0 用默认
    std::string scenarioPath;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--evaluate") {
            evaluate = true;
        } else if (arg.rfind("--duration=", 0) == 0) {
            duration = std::strtod(arg.substr(11).c_str(), nullptr);
            if (duration <= 0.0) {
                std::cerr << "Invalid duration: " << arg << "\n";
                return 2;
            }
        } else if (arg.rfind("--constraint=", 0) == 0) {
            const std::string v = arg.substr(13);
            if (v == "soft") cmode = ato::MpcConstraintMode::Soft;
            else if (v == "projected") cmode = ato::MpcConstraintMode::Projected;
            else if (v == "activeset") cmode = ato::MpcConstraintMode::ActiveSet;
            else {
                std::cerr << "Unknown constraint mode: " << v << "\n";
                return 2;
            }
        } else if (arg.rfind("--vref-margin=", 0) == 0) {
            vrefMargin = std::strtod(arg.substr(14).c_str(), nullptr);
        } else if (arg.rfind("--vref-margin-far=", 0) == 0) {
            vrefMarginFar = std::strtod(arg.substr(19).c_str(), nullptr);
        } else if (arg.rfind("--vref-far-dist=", 0) == 0) {
            vrefFarDist = std::strtod(arg.substr(16).c_str(), nullptr);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            scenarioPath = arg;
        }
    }

    if (scenarioPath.empty()) {
        printUsage(argv[0]);
        return 2;
    }

    std::ifstream input(scenarioPath);
    if (!input) {
        std::cerr << "failed to open scenario: " << scenarioPath << "\n";
        return 1;
    }

    // 解析场景到时间序列。
    std::vector<ScenarioRow> rows;
    double initialSpeed = 0.0;  // 首行 train_speed，作为 plant 初值
    std::string line;
    bool firstLine = true;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        if (firstLine) {
            firstLine = false;
            if (line.find("time_s") != std::string::npos) continue;
        }
        const std::vector<std::string> cells = splitCsvLine(line);
        ScenarioRow r;
        r.time = numberAt(cells, 0);
        const double seedSpeed = numberAt(cells, 1);
        if (rows.empty()) {
            initialSpeed = seedSpeed;  // 仅首行速度作为闭环初值
        }
        r.ebiSpeed = numberAt(cells, 2, 2200.0);
        r.runLevelSpeed = numberAt(cells, 3, 2200.0);
        r.distanceToStop = numberAt(cells, 4, ato::kInvalidDistance);
        r.grade = numberAt(cells, 5);
        r.phase = phaseAt(cells, 6);
        r.trainStopped = boolAt(cells, 7);
        r.stopPointId = intAt(cells, 8);
        r.restrictions = parseRestrictions(cells, 9, 10, 11, r.grade);
        (void)seedSpeed;
        rows.push_back(r);
    }
    if (rows.empty()) {
        std::cerr << "empty scenario: " << scenarioPath << "\n";
        return 1;
    }

    ato::ControlConfig cfg = defaultConfig();
    // 应用约束模式与 vref 裕度（仅 MPC 路径生效）。
    cfg.mpc.constraintMode = cmode;
    if (vrefMargin > 0.0) cfg.mpc.adaptiveBrakeMargin = vrefMargin;
    if (vrefMarginFar > 0.0) cfg.mpc.adaptiveBrakeMarginFar = vrefMarginFar;
    if (vrefFarDist > 0.0) cfg.mpc.adaptiveBrakeFarDist = vrefFarDist;
    ato::ControlLoop loop(cfg);
    ato::VehiclePlant plant(cfg.mpc.vehicle);
    const double effMass = cfg.mpc.vehicle.mass * (1.0 + cfg.mpc.vehicle.rotaryFactor);

    // 闭环状态：速度 cm/s、位置 cm。初值取首行场景速度。
    double simSpeed = initialSpeed;
    double simPosition = 0.0;

    // 停车点：首行 distance_to_stop 处（若有效）。停车场景据此判定停车。
    const bool hasStop = rows.front().distanceToStop > 0.0
                         && rows.front().distanceToStop < ato::kInvalidDistance;
    const double stopPositionCm = hasStop ? rows.front().distanceToStop : 0.0;
    const double expectedStopTimeS = rows.back().time;

    // 固定 200ms 控制周期，与 ATO 主循环及 MPC 预测时域一致。
    const double dt = 0.2;
    // 仿真终止时刻：
    //   停车场景：场景末时刻 + 120s 余量，让列车自然停下；
    //   巡航场景：max(场景末时刻, --duration)，--duration 让列车真正加速到巡航速度。
    double tEnd;
    if (hasStop) {
        tEnd = rows.back().time + 120.0;
    } else {
        tEnd = rows.back().time;
        if (duration > 0.0) tEnd = std::max(tEnd, duration);
    }

    std::cout << "time_s,train_speed,target_speed,cur_max_sbv,parking_brake_acc,"
              << "pid_demand,mpc_force,mpc_solve_ms,direction,analog_output,"
              << "traction,brake,brake_hold,stop_adjust,state\n";

    std::vector<ato::TrajectoryPoint> traj;
    traj.reserve(2048);

    double t = 0.0;
    while (t <= tEnd + 1e-9) {
        const ScenarioRow sc = sampleAt(rows, t);

        ato::ControlInput in;
        in.trainSpeed = simSpeed;                                   // 闭环反馈
        in.ebiSpeed = sc.ebiSpeed;
        in.runLevelSpeed = sc.runLevelSpeed;
        in.distanceToStop = hasStop ? (stopPositionCm - simPosition)
                                    : ato::kInvalidDistance;
        in.grade = sc.grade;
        in.phase = sc.phase;
        in.trainStopped = sc.trainStopped;
        in.stopPointId = sc.stopPointId;
        in.restrictions = sc.restrictions;
        in.currentPositionCm = simPosition;                         // 闭环位置反馈

        const ato::ControlOutput out = loop.step(in);

        // 控制器输出 -> 作用在 plant 上的力(N)。两条路径都已在 control_loop
        // 填好 out.appliedForce（MPC=首步预测力，PID=加速度需求反推），统一取用。
        double force = out.appliedForce;

        // 过冲保护：一旦越过停车点，目标速度本应归零，但距离变负会使
        // 目标速度模块误判为无停车点而回到巡航限速。此处直接施加最大
        // 制动直至停车，保证仿真必然收敛并给出真实的过冲量。
        const bool overshot = hasStop && simPosition >= stopPositionCm;
        if (overshot) {
            force = -plant.maxBrakeForce(simSpeed * 0.01);
        }

        // Plant 推进一步
        ato::VehicleState vs;
        vs.position = simPosition * 0.01;   // cm -> m
        vs.speed = simSpeed * 0.01;         // cm/s -> m/s
        vs.grade = sc.grade * 1e-3;         // ‰ -> rad
        const auto step = plant.step(vs, force);
        simSpeed = step.state.speed * 100.0;
        simPosition = step.state.position * 100.0;

        ato::TrajectoryPoint tp;
        tp.time = t;
        tp.trainSpeed = simSpeed;
        tp.targetSpeed = out.targetSpeed;
        tp.position = simPosition;
        tp.acceleration = step.state.acceleration * 100.0;
        tp.force = force;
        tp.analogOutput = out.command.analogOutput;
        tp.direction = static_cast<int>(out.command.direction);
        tp.solveTimeMs = out.mpc.solveTimeMs;
        tp.ebiSpeed = sc.ebiSpeed;                    // EBI 包络，用于超 EBI 统计
        tp.maxRecommendedSpeed = out.target.curMaxSbv;  // SBI 包络(最高推荐速度)
        traj.push_back(tp);

        std::cout << std::fixed << std::setprecision(3)
                  << t << ','
                  << simSpeed << ','
                  << out.targetSpeed << ','
                  << out.target.curMaxSbv << ','
                  << out.parking.brakeAcc << ','
                  << 0.0 << ','                           // pid_demand(保留列名,无PID填0)
                  << force << ','                          // mpc_force
                  << tp.solveTimeMs << ','
                  << ato::toString(out.command.direction) << ','
                  << out.command.analogOutput << ','
                  << (out.command.bits.traction ? 1 : 0) << ','
                  << (out.command.bits.brake ? 1 : 0) << ','
                  << (out.command.bits.brakeHold ? 1 : 0) << ','
                  << out.autoAdjust.currentAdjustValue << ','
                  << static_cast<int>(out.autoAdjust.state) << '\n';

        // 停车判定：有停车点且列车已近停（含轻微过冲）。
        if (hasStop && simSpeed < 5.0) {
            break;
        }
        // 巡航场景跑到 tEnd（含 --duration）即可，由 while 条件自然终止。
        t += dt;
    }

    if (evaluate && !traj.empty()) {
        const auto m = ato::Evaluator::evaluate(scenarioPath, traj,
            stopPositionCm, expectedStopTimeS, dt);
        std::cout << "# Evaluation\n";
        std::cout << "# scenario,stop_err_cm,stop_time_err_s,speed_rmse_cmps,"
                  << "speed_max_err_cmps,jerk_rms_cmps3,jerk_max_cmps3,"
                  << "dir_switches,energy_j,avg_solve_ms,max_solve_ms,steps,"
                  << "run_time_s,traction_energy_j,brake_energy_j,"
                  << "over_ebi_steps,over_ebi_max_cmps,over_sbi_steps,over_sbi_max_cmps,"
                  << "accel_rms_cmps2,accel_max_cmps2,trac_brake_switches\n";
        std::cout << "# " << ato::Evaluator::toCsvRow(m) << "\n";
    }

    return 0;
}
