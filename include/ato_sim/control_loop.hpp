#pragma once

#include "ato_sim/auto_adjust.hpp"
#include "ato_sim/constraint_provider.hpp"
#include "ato_sim/mpc_controller.hpp"
#include "ato_sim/output_transform.hpp"
#include "ato_sim/parking_brake.hpp"
#include "ato_sim/target_speed.hpp"

namespace ato {

class VehiclePlant;  // 前向声明，避免 include 重

// 控制闭环总配置。每个子结构体只包含算法参数，不包含通信、业务和设备状态。
// 注：本仿真包仅实现 MPC 控制器；PID 作为对比基准保留在 scenarios/pid_vs_mpc_report
// 静态结果中，不再提供运行时 PID 路径。
struct ControlConfig {
    ParkingBrakeConfig parking;
    TargetSpeedConfig target;
    EbSbConfig ebSb;
    OutputConfig output;
    AutoAdjustConfig autoAdjust;
    MpcConfig mpc;
};

// 一个 200ms 控制周期的仿真输入。调用方负责把外部数据准备好：
//   - ebiSpeed/runLevelSpeed 来自上游 ATP/运行等级
//   - restrictions 来自仿真线路/限速/MA 数据
//   - phase/trainStopped 用于选择巡航或停车算法
struct ControlInput {
    double trainSpeed = 0.0;
    double ebiSpeed = 0.0;
    double runLevelSpeed = 0.0;
    double distanceToStop = kInvalidDistance;
    double grade = 0.0;
    DrivePhase phase = DrivePhase::Cruise;
    bool trainStopped = false;
    int stopPointId = 0;
    std::vector<SpeedRestriction> restrictions;
    // 当前列车位置（cm）。闭环仿真由调用方维护并传入，MPC 约束生成据此
    // 推算预测时域内每步位置；开环回放时留 0 即可。
    double currentPositionCm = 0.0;
};

// 一个周期的完整算法输出，保留各子模块中间量，方便做仿真曲线和源码对拍。
struct ControlOutput {
    ParkingBrakeResult parking;
    TargetSpeedResult target;
    MpcPrediction mpc;
    OutputCommand command;
    AutoAdjustOutput autoAdjust;
    double targetSpeed = 0.0;
    // 车辆状态（Plant 仿真后）
    double simulatedSpeed = 0.0;     // cm/s
    double simulatedPosition = 0.0;  // cm
    double appliedForce = 0.0;       // N
};

// 独立仿真闭环。调用 step() 一次代表原工程主循环的一个 ATO 控制周期：
// 停车制动率 -> 目标速度 -> MPC -> 输出整形 -> 停车精度修正。
class ControlLoop {
public:
    explicit ControlLoop(ControlConfig config);

    // 主循环：和原工程保持纯函数语义，input 完全由调用方给定。
    ControlOutput step(const ControlInput& in);
    void reset();

    // 启用 Plant 仿真。enablePlant() 后，step() 内部会使用 vehicle_plant 推进状态。
    void enablePlant() { plantEnabled_ = true; }
    bool plantEnabled() const { return plantEnabled_; }
    double simulatedSpeed() const { return simSpeed_; }
    double simulatedPosition() const { return simPosition_; }
    void setSimulatedState(double speedCmps, double positionCm) {
        simSpeed_ = speedCmps;
        simPosition_ = positionCm;
    }

private:
    ControlConfig config_;
    OutputTransformer output_;
    AutoAdjuster autoAdjust_;
    MpcController mpc_;
    class VehiclePlant* plant_ = nullptr;  // 延迟构造，避免包含重
    bool plantEnabled_ = false;
    double simSpeed_ = 0.0;
    double simPosition_ = 0.0;
    double lastGrade_ = 0.0;

    // 上一周期目标速度，用于停车目标制动率触发判断。
    double lastTargetSpeed_ = 0.0;

    // 内部辅助：基于当前 step 的输出推进 plant
    void advancePlant(ControlInput& in, const ControlOutput& out);
};

}  // namespace ato
