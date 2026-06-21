#pragma once

#include <string>
#include <vector>

namespace ato {

// 单步仿真轨迹采样点。
struct TrajectoryPoint {
    double time = 0.0;            // s
    double trainSpeed = 0.0;      // cm/s
    double targetSpeed = 0.0;     // cm/s
    double position = 0.0;        // cm
    double acceleration = 0.0;    // cm/s^2
    double force = 0.0;           // N
    double analogOutput = 0.0;
    int direction = 0;            // 0=coast, 1=traction, 2=brake
    double jerk = 0.0;            // cm/s^3
    double solveTimeMs = 0.0;     // MPC 求解耗时（PID 路径为 0）
    double ebiSpeed = 0.0;            // 该步 EBI 速度（cm/s），用于超 EBI 统计
    double maxRecommendedSpeed = 0.0; // 该步最高推荐速度(SBI 包络, cm/s)
};

// 仿真评估指标
struct EvaluationMetrics {
    std::string scenario;

    // 停车精度：停车点（最后一帧）位置误差（cm），绝对值。
    double stopError = 0.0;
    double stopTimeError = 0.0;       // s，相对期望停车时刻

    // 速度跟踪 RMSE（cm/s）
    double speedRmse = 0.0;
    double speedMaxError = 0.0;

    // Jerk（cm/s^3）
    double jerkRms = 0.0;
    double jerkMax = 0.0;

    // 牵引/制动切换次数
    int directionSwitches = 0;

    // 能耗指标：∫|F|*v dt，单位 N·m = J
    double energy = 0.0;
    // 能耗分项：牵引耗能（正向力做功）、制动耗能（负向力绝对值，可视为再生上限）。
    double tractionEnergy = 0.0;  // J
    double brakeEnergy = 0.0;     // J

    // 控制层时间
    double totalControlTime = 0.0;

    // 求解器时间（仅 MPC 有意义）
    double avgSolveTimeMs = 0.0;
    double maxSolveTimeMs = 0.0;

    int totalSteps = 0;

    // ---- 运行效率 ----
    // 区间运行时间：从首帧到末帧的总时长（s）。
    double runTimeS = 0.0;

    // ---- 安全：超速统计 ----
    // 超 EBI：列车速度越过紧急制动包络的周期数与最大越界量（cm/s）。
    int overspeedEbiSteps = 0;
    double overspeedEbiMaxCmps = 0.0;
    // 超 SBI(最高推荐速度 curMaxSbv)：周期数与最大越界量（cm/s）。
    int overspeedSbiSteps = 0;
    double overspeedSbiMaxCmps = 0.0;

    // ---- 舒适度 ----
    // 加速度 RMS / 最大（cm/s^2）。
    double accelRms = 0.0;
    double accelMax = 0.0;
    // 牵引↔制动直接切换次数（traction<->brake，跳过 coast 的抖动）。
    int tractionBrakeSwitches = 0;
};

// 评估器：输入完整轨迹，输出指标。
class Evaluator {
public:
    // 计算所有指标。
    static EvaluationMetrics evaluate(
        const std::string& scenario,
        const std::vector<TrajectoryPoint>& traj,
        double stopPointCm = 0.0,           // 期望停车点 cm；0 表示不评估停车精度
        double expectedStopTimeS = 0.0,     // 期望停车时刻 s；<=0 不评估
        double sampleTimeS = 0.2);

    // 把两个评估结果合成对比行（CSV）
    static std::string toCsvRow(const EvaluationMetrics& m);

    // 多个场景聚合（输出 Markdown 表格）
    static std::string toMarkdownTable(
        const std::vector<EvaluationMetrics>& pidMetrics,
        const std::vector<EvaluationMetrics>& mpcMetrics);

    // 把指标写到 CSV 文件
    static bool writeCsv(const std::string& path,
                         const std::vector<EvaluationMetrics>& metrics);
};

}  // namespace ato
