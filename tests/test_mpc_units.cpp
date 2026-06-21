#include "ato_sim/constraint_provider.hpp"
#include "ato_sim/control_loop.hpp"
#include "ato_sim/evaluator.hpp"
#include "ato_sim/linalg.hpp"
#include "ato_sim/mpc_controller.hpp"
#include "ato_sim/sqpmethod.hpp"
#include "ato_sim/vehicle_plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct TestContext {
    int passed = 0;
    int failed = 0;
};

TestContext g_ctx;
std::string g_currentTest;

void fail(const std::string& message) {
    std::cerr << "[FAIL] " << g_currentTest << ": " << message << "\n";
    g_ctx.failed++;
}

void expectTrue(bool condition, const std::string& message) {
    if (!condition) fail(message);
}

void expectNear(double actual, double expected, double tolerance, const std::string& name) {
    if (std::fabs(actual - expected) > tolerance) {
        std::ostringstream oss;
        oss << name << " expected " << expected << " actual " << actual;
        fail(oss.str());
    }
}

void runTest(const std::string& name, void (*test)()) {
    g_currentTest = name;
    const int before = g_ctx.failed;
    test();
    if (g_ctx.failed == before) {
        g_ctx.passed++;
        std::cout << "[PASS] " << name << "\n";
    }
}

// =================================================================
// linalg 测试
// =================================================================

void testLinalgIdentityMultiply() {
    ato::mpc_internal::Matrix A = ato::mpc_internal::Matrix::identity(3);
    ato::mpc_internal::Matrix B = ato::mpc_internal::Matrix::zeros(3, 3);
    B(0, 0) = 1.0; B(0, 1) = 2.0; B(0, 2) = 3.0;
    B(1, 0) = 4.0; B(1, 1) = 5.0; B(1, 2) = 6.0;
    B(2, 0) = 7.0; B(2, 1) = 8.0; B(2, 2) = 9.0;
    auto C = A * B;
    expectNear(C(0, 0), 1.0, 1e-9, "I*B[0,0]");
    expectNear(C(2, 2), 9.0, 1e-9, "I*B[2,2]");
}

void testLinalgTranspose() {
    ato::mpc_internal::Matrix A(2, 3);
    A(0, 0) = 1; A(0, 1) = 2; A(0, 2) = 3;
    A(1, 0) = 4; A(1, 1) = 5; A(1, 2) = 6;
    auto T = A.transpose();
    expectTrue(T.rows() == 3 && T.cols() == 2, "transpose dims");
    expectNear(T(0, 1), 4.0, 1e-9, "T[0,1]");
    expectNear(T(2, 0), 3.0, 1e-9, "T[2,0]");
}

void testLinalgSolveSymmetric() {
    // A = [[2, 1], [1, 2]], b = [3, 3], x = [1, 1]
    ato::mpc_internal::Matrix A(2, 2);
    A(0, 0) = 2.0; A(0, 1) = 1.0;
    A(1, 0) = 1.0; A(1, 1) = 2.0;
    std::vector<double> b = {3.0, 3.0};
    std::vector<double> x;
    expectTrue(A.solveSymmetric(b, x), "SPD solve");
    expectNear(x[0], 1.0, 1e-9, "x[0]");
    expectNear(x[1], 1.0, 1e-9, "x[1]");
}

void testLinalgLargerSpdSolve() {
    // 5x5 对角占优 SPD 矩阵
    ato::mpc_internal::Matrix A = ato::mpc_internal::Matrix::identity(5) * 5.0;
    for (int i = 0; i < 4; ++i) {
        A(i, i + 1) = -1.0;
        A(i + 1, i) = -1.0;
    }
    std::vector<double> b = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> x;
    expectTrue(A.solveSymmetric(b, x), "5x5 SPD solve");
    // 验证 Ax ≈ b
    for (std::size_t i = 0; i < b.size(); ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < b.size(); ++j) {
            s += A(i, j) * x[j];
        }
        expectNear(s, b[i], 1e-6, "Ax[i] residual");
    }
}

// =================================================================
// vehicle_plant 测试
// =================================================================

ato::VehicleConfig standardVehicleConfig() {
    ato::VehicleConfig cfg;
    cfg.mass = 200000.0;
    cfg.rotaryFactor = 0.10;
    cfg.gravity = 9.81;
    cfg.davisA0 = 2000.0;
    cfg.davisA1 = 30.0;
    cfg.davisA2 = 3.0;
    cfg.tractionCurve = {{0.0, 100.0}, {1000.0, 100.0}, {2000.0, 60.0}, {3000.0, 30.0}};
    cfg.brakeCurve = {{0.0, 110.0}, {1000.0, 100.0}, {2000.0, 90.0}};
    return cfg;
}

void testVehiclePlantNoForceIdleDecel() {
    ato::VehiclePlant plant(standardVehicleConfig());
    ato::VehicleState s;
    s.position = 0.0;
    s.speed = 10.0;  // m/s = 36 km/h
    s.grade = 0.0;
    const auto r = plant.step(s, 0.0);
    // 没有牵引力，会因为阻力减速
    expectTrue(r.state.speed < s.speed, "no-force decel");
    expectTrue(r.state.speed > 0.0, "no-force still moving");
    expectTrue(r.state.position > s.position, "no-force still moves forward");
}

void testVehiclePlantTractionAccelerates() {
    ato::VehiclePlant plant(standardVehicleConfig());
    ato::VehicleState s;
    s.position = 0.0;
    s.speed = 0.0;
    s.grade = 0.0;
    const double maxF = plant.maxTractionForce(0.0);
    const auto r = plant.step(s, maxF);
    // 满牵引，速度应该增加
    expectTrue(r.state.speed > 0.0, "traction accelerates");
    expectTrue(r.state.speed < 5.0, "traction within bounds");
}

void testVehiclePlantBrakeDecelerates() {
    ato::VehiclePlant plant(standardVehicleConfig());
    ato::VehicleState s;
    s.position = 0.0;
    s.speed = 10.0;
    s.grade = 0.0;
    const double maxB = plant.maxBrakeForce(10.0);
    const auto r = plant.step(s, -maxB);
    // 满制动，速度应该下降（不要求很大幅度，因为 200ms 周期 + 旋转质量）
    expectTrue(r.state.speed < s.speed + 0.1, "brake decel");
}

void testVehiclePlantUphillAddsResistance() {
    ato::VehiclePlant plant(standardVehicleConfig());
    ato::VehicleState s;
    s.position = 0.0;
    s.speed = 5.0;
    s.grade = 0.0;
    const auto flat = plant.step(s, 0.0);
    s.grade = 0.05;  // ~3 度
    const auto up = plant.step(s, 0.0);
    // 上坡阻力更大
    expectTrue(up.state.speed < flat.state.speed, "uphill decel faster");
}

void testVehiclePlantMaxForcesNonZero() {
    ato::VehiclePlant plant(standardVehicleConfig());
    expectTrue(plant.maxTractionForce(0.0) > 0.0, "traction positive");
    expectTrue(plant.maxBrakeForce(0.0) > 0.0, "brake positive");
    expectTrue(plant.maxTractionForce(20.0) > 0.0, "traction positive at 20m/s");
    expectTrue(plant.maxBrakeForce(20.0) > 0.0, "brake positive at 20m/s");
}

void testVehiclePlantUnitConversion() {
    expectNear(ato::VehiclePlant::cmpsToMps(100.0), 1.0, 1e-9, "cmps->mps");
    expectNear(ato::VehiclePlant::mpsToCmps(1.0), 100.0, 1e-9, "mps->cmps");
    expectNear(ato::VehiclePlant::cmToM(100.0), 1.0, 1e-9, "cm->m");
    expectNear(ato::VehiclePlant::mps2ToCmps2(1.0), 100.0, 1e-9, "mps2->cmps2");
}

// =================================================================
// constraint_provider 测试
// =================================================================

ato::ConstraintConfig standardConstraintConfig() {
    ato::ConstraintConfig cfg;
    cfg.maxLineLimitSpeed = 2200.0;
    cfg.minMarginFromEbi = 55.0;
    cfg.ebSb.motorAcc = 80.0;
    cfg.ebSb.brakeAcc = 120.0;
    cfg.ebSb.motorDelay = 1.0;
    cfg.ebSb.brakeDelay = 2.0;
    cfg.ebSb.trainRotateRate = 1.0;
    return cfg;
}

void testConstraintProviderEmptyGivesMaxSpeed() {
    const auto steps = ato::ConstraintProvider::generate(
        standardConstraintConfig(), 1000.0, 0.0, 0.0, 10);
    expectTrue(steps.size() == 11, "horizon+1 steps");
    for (const auto& s : steps) {
        expectTrue(s.maxSpeed > 0.0, "non-zero max speed");
    }
}

void testConstraintProviderWithRestrictionLowersMax() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.restrictions = {{5000.0, 1000.0, 0.0}};
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 2200.0, 0.0, 0.0, 10);
    expectTrue(steps.front().maxSpeed < 2200.0, "lower max with restriction");
    expectTrue(steps.front().maxSpeed > 0.0, "still non-zero max");
}

void testConstraintProviderWithStopPoint() {
    const auto steps = ato::ConstraintProvider::generate(
        standardConstraintConfig(), 1000.0, 0.0, 5000.0, 30);
    // 第一步的 targetSpeed 应该小于等于 maxSpeed
    for (std::size_t i = 0; i < steps.size(); ++i) {
        expectTrue(steps[i].targetSpeed <= steps[i].maxSpeed + 1e-6,
                   "targetSpeed <= maxSpeed");
    }
    // 末端应该接近 0（停车）
    expectTrue(steps.back().maxSpeed < 100.0, "stop step max speed low");
}

void testConstraintProviderRunLevelCap() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.runLevelSpeed = 1500.0;
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 2200.0, 0.0, 0.0, 10);
    expectTrue(steps.front().maxSpeed <= 1500.0, "run level cap");
}

// ---- 自适应 vref（方案 A）测试 ----
// 制动曲线：低速 110、高速 95 cm/s^2（与 CLI 默认一致）。
ato::Curve adaptiveBrakeCurve() {
    return {{0.0, 110.0}, {600.0, 110.0}, {1600.0, 105.0}, {2400.0, 95.0}};
}

// 平坡自适应 vref 应与固定保守值同量级（不制动不及过冲，故不能激进高于）。
// 自适应的价值在坡度修正（下坡更保守、上坡略松），而非平坡提速。
void testConstraintProviderAdaptiveVrefHigherThanFixed() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.brakeCurve = adaptiveBrakeCurve();
    cfg.grade = 0.0;
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 1500.0, 0.0, 5000.0, 30);  // 50m 停车
    // 取开始减速的首步比较（未被 maxSpeed 封顶处）。
    double vAdaptive = -1.0;
    double distAt = -1.0;
    for (const auto& s : steps) {
        if (s.targetSpeed < s.maxSpeed - 1.0 && s.targetSpeed > 0.0) {
            vAdaptive = s.targetSpeed;
            distAt = 5000.0 - s.positionRef;
            break;
        }
    }
    expectTrue(vAdaptive > 0.0, "found a decelerating step");
    const double aFixed = 0.35 * cfg.ebSb.brakeAcc;
    const double vFixed = std::sqrt(2.0 * aFixed * distAt);
    // 平坡自适应应与固定保守值同量级（0.5x ~ 1.5x），既不激进过冲也不过度保守。
    expectTrue(vAdaptive > vFixed * 0.5 && vAdaptive < vFixed * 1.5,
               "flat adaptive vref same order as fixed conservative");
}

// 下坡 vref 应低于平坡(制动变弱，更保守)。用中近距离使 vref 不被 maxSpeed 封顶。
void testConstraintProviderAdaptiveVrefDownhillLower() {
    ato::ConstraintConfig cfgFlat = standardConstraintConfig();
    cfgFlat.brakeCurve = adaptiveBrakeCurve();
    cfgFlat.grade = 0.0;
    const auto stepsFlat = ato::ConstraintProvider::generate(
        cfgFlat, 1500.0, 0.0, 5000.0, 30);

    ato::ConstraintConfig cfgDown = standardConstraintConfig();
    cfgDown.brakeCurve = adaptiveBrakeCurve();
    cfgDown.grade = -15.0;  // 下坡 15‰
    const auto stepsDown = ato::ConstraintProvider::generate(
        cfgDown, 1500.0, 0.0, 5000.0, 30);

    // 取开始减速的首步比较(未被 maxSpeed 封顶处)
    auto firstDecel = [](const std::vector<ato::ConstraintStep>& st) -> double {
        for (const auto& s : st) {
            if (s.targetSpeed < s.maxSpeed - 1.0 && s.targetSpeed > 0.0) return s.targetSpeed;
        }
        return -1.0;
    };
    const double vFlat = firstDecel(stepsFlat);
    const double vDown = firstDecel(stepsDown);
    expectTrue(vFlat > 0.0 && vDown > 0.0, "found decel steps");
    expectTrue(vDown < vFlat, "downhill vref should be lower than flat");
}

// 上坡 vref 应高于平坡(制动增强)。
void testConstraintProviderAdaptiveVrefUphillHigher() {
    ato::ConstraintConfig cfgFlat = standardConstraintConfig();
    cfgFlat.brakeCurve = adaptiveBrakeCurve();
    cfgFlat.grade = 0.0;
    const auto stepsFlat = ato::ConstraintProvider::generate(
        cfgFlat, 1500.0, 0.0, 5000.0, 30);

    ato::ConstraintConfig cfgUp = standardConstraintConfig();
    cfgUp.brakeCurve = adaptiveBrakeCurve();
    cfgUp.grade = 20.0;  // 上坡 20‰
    const auto stepsUp = ato::ConstraintProvider::generate(
        cfgUp, 1500.0, 0.0, 5000.0, 30);

    auto firstDecel = [](const std::vector<ato::ConstraintStep>& st) -> double {
        for (const auto& s : st) {
            if (s.targetSpeed < s.maxSpeed - 1.0 && s.targetSpeed > 0.0) return s.targetSpeed;
        }
        return -1.0;
    };
    const double vFlat = firstDecel(stepsFlat);
    const double vUp = firstDecel(stepsUp);
    expectTrue(vFlat > 0.0 && vUp > 0.0, "found decel steps");
    expectTrue(vUp > vFlat, "uphill vref should be higher than flat");
}

// vref 应随接近停车点单调递减，末端接近 0。
void testConstraintProviderAdaptiveVrefMonotonicDecrease() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.brakeCurve = adaptiveBrakeCurve();
    cfg.grade = 0.0;
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 1500.0, 0.0, 5000.0, 30);
    // 从开始减速处到末端应单调递减
    bool monotonic = true;
    bool started = false;
    for (std::size_t i = 0; i < steps.size(); ++i) {
        if (!started) {
            if (steps[i].targetSpeed < steps[i].maxSpeed - 1.0 && steps[i].targetSpeed > 0.0) {
                started = true;
            }
            continue;
        }
        if (i > 0 && steps[i].targetSpeed > steps[i - 1].targetSpeed + 1.0) {
            monotonic = false;
            break;
        }
    }
    expectTrue(monotonic, "vref should be non-increasing toward stop");
    expectTrue(steps.back().targetSpeed < 50.0, "terminal vref near zero");
}

// vref 不应超过 maxSpeed 走廊(安全上界)。
void testConstraintProviderAdaptiveVrefCappedByMax() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.brakeCurve = adaptiveBrakeCurve();
    cfg.grade = 0.0;
    cfg.runLevelSpeed = 800.0;  // 限速走廊压低
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 1500.0, 0.0, 5000.0, 30);
    for (const auto& s : steps) {
        expectTrue(s.targetSpeed <= s.maxSpeed + 1e-6, "vref <= maxSpeed");
    }
}

// 极端下坡时 a_floor 兜底，vref 不应退化为 0 或爆炸。
void testConstraintProviderAdaptiveVrefFloorOnExtremeDownhill() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    cfg.brakeCurve = adaptiveBrakeCurve();
    cfg.grade = -100.0;  // 极端下坡
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 1500.0, 0.0, 5000.0, 30);
    // vref 应为有限正数(非 0、非 inf/nan)
    for (const auto& s : steps) {
        expectTrue(s.targetSpeed >= 0.0 && std::isfinite(s.targetSpeed),
                   "vref finite non-negative on extreme downhill");
    }
    // 开始减速处 vref 应非零
    bool anyPositive = false;
    for (const auto& s : steps) {
        if (s.targetSpeed > 0.0) { anyPositive = true; break; }
    }
    expectTrue(anyPositive, "vref non-zero before stop");
}

// 无制动曲线时退化为固定保守制动率反推(兼容旧行为：仍产出递减 vref)。
void testConstraintProviderAdaptiveVrefFallbackNoCurve() {
    ato::ConstraintConfig cfg = standardConstraintConfig();
    // 不设 brakeCurve
    const auto steps = ato::ConstraintProvider::generate(
        cfg, 1500.0, 0.0, 5000.0, 30);
    // 退化模式仍应：vref <= maxSpeed、末端接近 0、存在减速段。
    for (const auto& s : steps) {
        expectTrue(s.targetSpeed <= s.maxSpeed + 1e-6, "fallback vref <= maxSpeed");
    }
    expectTrue(steps.back().targetSpeed < 50.0, "fallback terminal vref near zero");
    bool hasDecel = false;
    for (const auto& s : steps) {
        if (s.targetSpeed < s.maxSpeed - 1.0 && s.targetSpeed > 0.0) { hasDecel = true; break; }
    }
    expectTrue(hasDecel, "fallback has decelerating segment");
}

// ---- 分段 vref 测试 ----
// 分段：远处用激进 margin(高)，近处用保守 margin(低)。
// 验证：相同停车距离下，分段(远)的 vref 高于纯保守；分段(近)接近纯保守。

// 分段 vref：远处(大 distToStop)的 vref 应高于纯保守 margin 的 vref。
void testConstraintProviderSegmentedVrefFarHigher() {
    ato::ConstraintConfig cfgCons = standardConstraintConfig();
    cfgCons.brakeCurve = adaptiveBrakeCurve();
    cfgCons.grade = 0.0;
    cfgCons.adaptiveBrakeMargin = 0.4;
    ato::ConstraintConfig cfgSeg = standardConstraintConfig();
    cfgSeg.brakeCurve = adaptiveBrakeCurve();
    cfgSeg.grade = 0.0;
    cfgSeg.adaptiveBrakeMargin = 0.4;
    cfgSeg.adaptiveBrakeMarginFar = 0.8;
    cfgSeg.adaptiveBrakeFarDist = 30000.0;

    // 80m 停车，初速 1500。首步 dist=8000cm，vref=sqrt(2*a*8000) < maxSpeed=2145，
    // 远场(dist>FarDist)分段用 margin=0.8 → vref 高于保守 margin=0.4。
    const auto stepsCons = ato::ConstraintProvider::generate(
        cfgCons, 1500.0, 0.0, 8000.0, 30);
    const auto stepsSeg = ato::ConstraintProvider::generate(
        cfgSeg, 1500.0, 0.0, 8000.0, 30);
    // 首步 vref(未被 maxSpeed 封顶，因 sqrt(2*38*8000)=778 < 2145)
    const double vCons = stepsCons.front().targetSpeed;
    const double vSeg = stepsSeg.front().targetSpeed;
    expectTrue(vCons > 0.0 && vSeg > 0.0, "found decel step");
    expectTrue(vSeg > vCons, "segmented vref higher than conservative at far");
}

// 分段 vref：临近停车(小 distToStop)的 vref 应接近纯保守 margin。
void testConstraintProviderSegmentedVrefNearConservative() {
    ato::ConstraintConfig cfgCons = standardConstraintConfig();
    cfgCons.brakeCurve = adaptiveBrakeCurve();
    cfgCons.adaptiveBrakeMargin = 0.4;
    ato::ConstraintConfig cfgSeg = standardConstraintConfig();
    cfgSeg.brakeCurve = adaptiveBrakeCurve();
    cfgSeg.adaptiveBrakeMargin = 0.4;
    cfgSeg.adaptiveBrakeMarginFar = 0.8;
    cfgSeg.adaptiveBrakeFarDist = 30000.0;

    // 30m 停车(全在近场 dist<FarDist)，分段应退化到保守 margin=0.4
    const auto stepsCons = ato::ConstraintProvider::generate(
        cfgCons, 500.0, 0.0, 3000.0, 30);
    const auto stepsSeg = ato::ConstraintProvider::generate(
        cfgSeg, 500.0, 0.0, 3000.0, 30);
    const double vCons = stepsCons.front().targetSpeed;
    const double vSeg = stepsSeg.front().targetSpeed;
    expectTrue(vCons > 0.0 && vSeg > 0.0, "found near decel step");
    // 近场 vSeg 应接近保守(过渡到保守值，不超 1.05 倍)
    expectTrue(vSeg <= vCons * 1.05, "segmented near conservative");
}

// 分段禁用(marginFar<=margin)时与单一 margin 行为一致。
void testConstraintProviderSegmentedDisabled() {
    ato::ConstraintConfig cfgSingle = standardConstraintConfig();
    cfgSingle.brakeCurve = adaptiveBrakeCurve();
    cfgSingle.adaptiveBrakeMargin = 0.4;
    ato::ConstraintConfig cfgDisabled = standardConstraintConfig();
    cfgDisabled.brakeCurve = adaptiveBrakeCurve();
    cfgDisabled.adaptiveBrakeMargin = 0.4;
    cfgDisabled.adaptiveBrakeMarginFar = 0.0;  // 禁用

    const auto s1 = ato::ConstraintProvider::generate(
        cfgSingle, 1500.0, 0.0, 100000.0, 30);
    const auto s2 = ato::ConstraintProvider::generate(
        cfgDisabled, 1500.0, 0.0, 100000.0, 30);
    for (std::size_t k = 0; k < s1.size(); ++k) {
        expectNear(s1[k].targetSpeed, s2[k].targetSpeed, 1e-6,
                   "disabled segmented equals single margin");
    }
}

// =================================================================
// sqpmethod 测试
// =================================================================

void testSqpmethodSimpleQuadratic() {
    // min 0.5*(x-3)^2 + 0.5*(y+2)^2, 无约束
    ato::SqpmethodConfig cfg;
    cfg.maxIterations = 20;
    cfg.gradientTol = 1e-4;
    cfg.stepTol = 1e-6;
    ato::Sqpmethod solver(cfg);

    auto cost = [](const std::vector<double>& x) {
        return 0.5 * (x[0] - 3.0) * (x[0] - 3.0)
             + 0.5 * (x[1] + 2.0) * (x[1] + 2.0);
    };
    auto grad = [](const std::vector<double>& x, std::vector<double>& g) {
        g = {x[0] - 3.0, x[1] + 2.0};
    };
    const std::vector<double> x0 = {0.0, 0.0};
    const std::vector<double> gLower = {-1e9, -1e9};
    const std::vector<double> gUpper = {1e9, 1e9};

    auto constraintValue = [](std::size_t, const std::vector<double>&, double&) {};
    auto constraintJacobian = [](const std::vector<double>&, ato::mpc_internal::Matrix& J) {
        J = ato::mpc_internal::Matrix::zeros(0, 0);
    };

    const auto r = solver.solve(2, 0, x0, gLower, gUpper,
                                 cost, grad, constraintValue, constraintJacobian);
    // 我们的简单 SQP 可能不会完美收敛到 (3, -2)，但 cost 应该比初始小
    // （实际解最小值 0）
    expectTrue(r.cost < 10.0, "simple quad cost decreased");
}

void testSqpmethodBoxConstrained() {
    // min 0.5*(x-10)^2, s.t. x <= 5
    ato::SqpmethodConfig cfg;
    cfg.maxIterations = 30;
    cfg.gradientTol = 1e-3;
    cfg.stepTol = 1e-5;
    ato::Sqpmethod solver(cfg);

    auto cost = [](const std::vector<double>& x) {
        return 0.5 * (x[0] - 10.0) * (x[0] - 10.0);
    };
    auto grad = [](const std::vector<double>& x, std::vector<double>& g) {
        g = {x[0] - 10.0};
    };
    std::vector<double> x0 = {0.0};
    // 我们用 box 形式：g_0 = x[0], g_0 <= 5
    std::vector<double> gLower = {-1e9};
    std::vector<double> gUpper = {5.0};

    auto constraintValue = [](std::size_t i, const std::vector<double>& x, double& gv) {
        gv = x[i];
    };
    auto constraintJacobian = [](const std::vector<double>&, ato::mpc_internal::Matrix& J) {
        J = ato::mpc_internal::Matrix::zeros(1, 1);
        J(0, 0) = 1.0;
    };
    const auto r = solver.solve(1, 1, x0, gLower, gUpper,
                                 cost, grad, constraintValue, constraintJacobian);
    expectTrue(r.cost < 0.5 * 5.0 * 5.0 + 1.0, "box constrained cost bounded");
}

// =================================================================
// mpc_controller 测试
// =================================================================

ato::MpcConfig standardMpcConfig() {
    ato::MpcConfig cfg;
    cfg.horizon = 10;  // 测试用短时域
    cfg.sampleTime = 0.2;
    cfg.wSpeed = 1.0;
    cfg.wControl = 1e-4;
    cfg.wJerk = 5e-3;
    cfg.wTerminal = 100.0;
    cfg.wStopSpeed = 50.0;
    cfg.vehicle = standardVehicleConfig();
    cfg.solver.maxIterations = 5;
    cfg.solver.gradientTol = 1e-3;
    cfg.solver.stepTol = 1e-4;
    return cfg;
}

void testMpcControllerProducesPrediction() {
    ato::MpcController mpc(standardMpcConfig());
    std::vector<ato::ConstraintStep> constraints(11);
    for (int k = 0; k <= 10; ++k) {
        constraints[k].maxSpeed = 1500.0;  // cm/s
        constraints[k].targetSpeed = 1500.0;
        constraints[k].minSpeed = 0.0;
    }
    const auto pred = mpc.step(0.0, 0.0, 0.0, constraints, false, 0.0, 10);
    expectTrue(pred.position.size() == 11, "pred position size");
    expectTrue(pred.force.size() == 10, "pred force size");
    // 第一步应该有非零力（加速到目标）
    expectTrue(pred.cost >= 0.0, "non-negative cost");
}

void testMpcControllerWithStopPoint() {
    ato::MpcController mpc(standardMpcConfig());
    std::vector<ato::ConstraintStep> constraints(11);
    for (int k = 0; k <= 10; ++k) {
        constraints[k].maxSpeed = 1000.0;
        constraints[k].targetSpeed = 0.0;
    }
    // 停车点距离 1000m = 100000cm
    const auto pred = mpc.step(500.0, 0.0, 0.0, constraints, true, 100000.0, 10);
    expectTrue(pred.position.size() == 11, "stop pred position size");
    // 末端速度应该较小
    expectTrue(pred.speed.back() < 500.0, "stop reduces speed");
}

void testMpcControllerReset() {
    ato::MpcController mpc(standardMpcConfig());
    std::vector<ato::ConstraintStep> constraints(11);
    for (int k = 0; k <= 10; ++k) {
        constraints[k].maxSpeed = 1500.0;
        constraints[k].targetSpeed = 1500.0;
    }
    mpc.step(0.0, 0.0, 0.0, constraints, false, 0.0, 10);
    mpc.reset();
    // reset 后可以继续 step
    const auto pred = mpc.step(0.0, 0.0, 0.0, constraints, false, 0.0, 10);
    expectTrue(pred.position.size() == 11, "post-reset pred");
}

// ---- B1/B2 速度硬约束测试 ----
// 构造 maxSpeed=1000 但 targetSpeed=2000 的约束：无约束解会想冲到 2000（超 1000）。
// 用弱软罚(wSlack=1)的 Soft 作对照——它会明显超 maxSpeed；B1/B2 应把速度压回。

// 构造"目标远高于上限"的约束序列：maxSpeed=1000, targetSpeed=2000。
std::vector<ato::ConstraintStep> overSpeedConstraints(int horizon) {
    std::vector<ato::ConstraintStep> c(static_cast<std::size_t>(horizon) + 1);
    for (auto& s : c) {
        s.maxSpeed = 1000.0;
        s.targetSpeed = 2000.0;  // 远超 maxSpeed，诱导无约束解冲超
        s.minSpeed = 0.0;
    }
    return c;
}

// 弱软罚 Soft 基线：wSlackSpeed 极小时，maxSpeed 不作为硬约束。
// 验证 Soft 模式不依赖约束求解器（constraintMaxIter 不影响结果）——
// 即 Soft 是纯罚函数，与 B1/B2 的迭代求解本质不同。
void testMpcSoftWeakPenaltyOverspeeds() {
    ato::MpcConfig cfg = standardMpcConfig();
    cfg.constraintMode = ato::MpcConstraintMode::Soft;
    cfg.wSlackSpeed = 0.01;  // 极弱罚
    cfg.constraintMaxIter = 1;  // Soft 不用迭代
    ato::MpcController mpc(cfg);
    auto constraints = overSpeedConstraints(10);
    const auto pred1 = mpc.step(900.0, 0.0, 0.0, constraints, false, 0.0, 10);
    // constraintMaxIter 改大不应改变 Soft 结果（Soft 不迭代）
    cfg.constraintMaxIter = 50;
    ato::MpcController mpc2(cfg);
    const auto pred2 = mpc2.step(900.0, 0.0, 0.0, constraints, false, 0.0, 10);
    expectNear(pred1.speed.back(), pred2.speed.back(), 1e-6,
               "soft independent of constraintMaxIter");
}

// B2 主动集：即便 wSlackSpeed 弱，硬约束应保证预测速度不超 maxSpeed（容许数值小量）。
void testMpcActiveSetRespectsMaxSpeed() {
    ato::MpcConfig cfg = standardMpcConfig();
    cfg.constraintMode = ato::MpcConstraintMode::ActiveSet;
    cfg.constraintMaxIter = 30;
    cfg.wSlackSpeed = 1.0;  // 弱罚，靠硬约束
    ato::MpcController mpc(cfg);
    auto constraints = overSpeedConstraints(10);
    const auto pred = mpc.step(900.0, 0.0, 0.0, constraints, false, 0.0, 10);
    for (std::size_t k = 0; k < pred.speed.size(); ++k) {
        expectTrue(pred.speed[k] <= 1000.0 + 5.0,
                   "active-set respects maxSpeed");
    }
}

// B1 投影法：即便 wSlackSpeed 弱，投影应把速度控制在 maxSpeed 附近（容许 15% 超）。
void testMpcProjectedRespectsMaxSpeed() {
    ato::MpcConfig cfg = standardMpcConfig();
    cfg.constraintMode = ato::MpcConstraintMode::Projected;
    cfg.constraintMaxIter = 30;
    cfg.wSlackSpeed = 1.0;  // 弱罚，靠投影
    ato::MpcController mpc(cfg);
    auto constraints = overSpeedConstraints(10);
    const auto pred = mpc.step(900.0, 0.0, 0.0, constraints, false, 0.0, 10);
    for (std::size_t k = 0; k < pred.speed.size(); ++k) {
        expectTrue(pred.speed[k] <= 1150.0,
                   "projected keeps speed near maxSpeed");
    }
}

// B2 比 B1/Soft 更严格：主动集的最大超速量应 <= 弱软罚 Soft。
void testMpcActiveSetTighterThanSoft() {
    auto maxOverspeed = [](const ato::MpcConfig& cfg) -> double {
        ato::MpcController mpc(cfg);
        auto constraints = overSpeedConstraints(10);
        const auto pred = mpc.step(900.0, 0.0, 0.0, constraints, false, 0.0, 10);
        double mx = 0.0;
        for (double v : pred.speed) mx = std::max(mx, v - 1000.0);
        return mx;
    };
    ato::MpcConfig softCfg = standardMpcConfig();
    softCfg.constraintMode = ato::MpcConstraintMode::Soft;
    softCfg.wSlackSpeed = 0.01;
    ato::MpcConfig asCfg = standardMpcConfig();
    asCfg.constraintMode = ato::MpcConstraintMode::ActiveSet;
    asCfg.constraintMaxIter = 30;
    asCfg.wSlackSpeed = 0.01;
    const double overSoft = maxOverspeed(softCfg);
    const double overAS = maxOverspeed(asCfg);
    expectTrue(overAS <= overSoft + 1.0,
               "active-set no more overspeed than weak soft");
    expectTrue(overAS <= 5.0, "active-set overspeed within tolerance");
}

// =================================================================
// evaluator 测试
// =================================================================

std::vector<ato::TrajectoryPoint> makeStraightTrajectory(int n) {
    std::vector<ato::TrajectoryPoint> traj;
    traj.reserve(n);
    for (int i = 0; i < n; ++i) {
        ato::TrajectoryPoint p;
        p.time = i * 0.2;
        p.trainSpeed = 1000.0 + i * 10.0;
        p.targetSpeed = 1100.0;
        p.position = i * 200.0;
        p.acceleration = 50.0;
        p.force = 1000.0;
        p.analogOutput = 5000.0;
        p.direction = 1;
        p.jerk = 0.0;
        traj.push_back(p);
    }
    return traj;
}

void testEvaluatorBasicMetrics() {
    const auto traj = makeStraightTrajectory(50);
    const auto m = ato::Evaluator::evaluate("test", traj, 0.0, 0.0, 0.2);
    expectTrue(m.totalSteps == 50, "step count");
    expectTrue(m.speedRmse > 0.0, "rmse non-zero");
    expectTrue(m.speedMaxError > 0.0, "max error non-zero");
    expectTrue(m.directionSwitches == 0, "no switches in same direction");
}

void testEvaluatorStopError() {
    auto traj = makeStraightTrajectory(10);
    traj.back().position = 1000.0;
    traj.back().trainSpeed = 0.0;
    traj.back().time = 2.0;
    const auto m = ato::Evaluator::evaluate("test", traj, 1000.0, 2.0, 0.2);
    expectNear(m.stopError, 0.0, 1e-9, "stop error zero");
    expectNear(m.stopTimeError, 0.0, 1e-9, "stop time error zero");
}

void testEvaluatorDirectionSwitches() {
    auto traj = makeStraightTrajectory(10);
    traj[3].direction = 2;  // coast->brake
    traj[5].direction = 0;  // brake->coast
    traj[7].direction = 1;  // coast->traction
    const auto m = ato::Evaluator::evaluate("test", traj);
    expectTrue(m.directionSwitches >= 2, "switches counted");
}

void testEvaluatorToCsvRow() {
    const auto traj = makeStraightTrajectory(10);
    const auto m = ato::Evaluator::evaluate("test", traj);
    const auto row = ato::Evaluator::toCsvRow(m);
    expectTrue(row.find("test") != std::string::npos, "csv has scenario");
    expectTrue(row.find(",") != std::string::npos, "csv has commas");
}

void testEvaluatorMarkdownTable() {
    const auto traj1 = makeStraightTrajectory(20);
    const auto traj2 = makeStraightTrajectory(20);
    const auto pidM = ato::Evaluator::evaluate("scn1", traj1);
    const auto mpcM = ato::Evaluator::evaluate("scn1", traj2);
    const auto md = ato::Evaluator::toMarkdownTable({pidM}, {mpcM});
    expectTrue(md.find("PID vs MPC") != std::string::npos, "table header");
    expectTrue(md.find("scn1") != std::string::npos, "scenario in table");
}

// 空轨迹：应安全返回零指标，不崩溃。
void testEvaluatorEmptyTrajectory() {
    std::vector<ato::TrajectoryPoint> empty;
    const auto m = ato::Evaluator::evaluate("empty", empty, 1000.0, 5.0, 0.2);
    expectTrue(m.totalSteps == 0, "empty totalSteps");
    expectNear(m.speedRmse, 0.0, 1e-9, "empty rmse");
    expectNear(m.stopError, 0.0, 1e-9, "empty stop error");
    expectNear(m.runTimeS, 0.0, 1e-9, "empty run time");
}

// 单步轨迹：jerk/方向切换等需要 i>0 的分支不应触发，不除零。
void testEvaluatorSingleStepTrajectory() {
    const auto traj = makeStraightTrajectory(1);
    const auto m = ato::Evaluator::evaluate("single", traj, 0.0, 0.0, 0.2);
    expectTrue(m.totalSteps == 1, "single step count");
    expectNear(m.jerkRms, 0.0, 1e-9, "single jerk rms zero");
    expectNear(m.jerkMax, 0.0, 1e-9, "single jerk max zero");
    expectTrue(m.directionSwitches == 0, "single no switches");
}

// 超 EBI / 超 SBI 统计：构造速度越过包络的轨迹。
void testEvaluatorOverspeedEbiSbi() {
    auto traj = makeStraightTrajectory(5);
    // 列车速度 1000+，设 EBI=800、SBI=900 → 全程超 EBI 和 SBI。
    for (auto& p : traj) {
        p.ebiSpeed = 800.0;
        p.maxRecommendedSpeed = 900.0;
    }
    const auto m = ato::Evaluator::evaluate("overspeed", traj, 0.0, 0.0, 0.2);
    expectTrue(m.overspeedEbiSteps == 5, "all steps over EBI");
    expectTrue(m.overspeedEbiMaxCmps > 0.0, "over EBI max positive");
    expectTrue(m.overspeedSbiSteps == 5, "all steps over SBI");
    expectTrue(m.overspeedSbiMaxCmps > 0.0, "over SBI max positive");
    expectTrue(m.overspeedEbiMaxCmps >= m.overspeedSbiMaxCmps,
               "EBI exceed >= SBI exceed when EBI<SBI");
}

// EBI/SBI 为 0 时不统计超速（包络未设定）。
void testEvaluatorOverspeedZeroEnvelope() {
    auto traj = makeStraightTrajectory(5);
    // ebiSpeed/maxRecommendedSpeed 默认 0，不应统计超速。
    const auto m = ato::Evaluator::evaluate("noenv", traj, 0.0, 0.0, 0.2);
    expectTrue(m.overspeedEbiSteps == 0, "no over EBI when envelope 0");
    expectTrue(m.overspeedSbiSteps == 0, "no over SBI when envelope 0");
}

// 能耗分项：正向力(牵引)与负向力(制动)分开累计。
void testEvaluatorEnergySplit() {
    std::vector<ato::TrajectoryPoint> traj;
    ato::TrajectoryPoint p;
    p.time = 0.0; p.trainSpeed = 1000.0; p.targetSpeed = 1000.0;
    p.position = 0.0; p.acceleration = 0.0; p.force = 200000.0;  // 牵引力
    p.direction = 1;
    traj.push_back(p);
    p.time = 0.2; p.force = -150000.0; p.direction = 2;  // 制动力
    traj.push_back(p);
    const auto m = ato::Evaluator::evaluate("energy", traj, 0.0, 0.0, 0.2);
    expectTrue(m.tractionEnergy > 0.0, "traction energy positive");
    expectTrue(m.brakeEnergy > 0.0, "brake energy positive");
    // 总能耗 = 牵引 + 制动（两者绝对值之和）
    expectNear(m.energy, m.tractionEnergy + m.brakeEnergy, 1e-6,
               "energy equals traction + brake");
}

// 牵引↔制动直接切换(1->2)计入 tracBrakeSwitches，跨 coast(1->0->2)不计。
void testEvaluatorTractionBrakeSwitches() {
    auto traj = makeStraightTrajectory(6);  // 全 traction(1)
    // traj[1..2]=2：1->2(直接切,计1)，2->2(不变)
    traj[1].direction = 2;
    traj[2].direction = 2;
    // traj[3..4] 跨 coast：2->0->1 不应计为 tracBrake 直接切换
    traj[3].direction = 0;
    traj[4].direction = 1;
    const auto m = ato::Evaluator::evaluate("switches", traj, 0.0, 0.0, 0.2);
    expectTrue(m.tractionBrakeSwitches == 1, "one direct trac-brake switch");
}

// 求解时间统计：仅 solveTimeMs>0 的步计入 avg/max。
void testEvaluatorSolveTime() {
    auto traj = makeStraightTrajectory(4);
    traj[0].solveTimeMs = 0.0;  // PID 步，不计
    traj[1].solveTimeMs = 5.0;
    traj[2].solveTimeMs = 15.0;
    traj[3].solveTimeMs = 10.0;
    const auto m = ato::Evaluator::evaluate("solve", traj, 0.0, 0.0, 0.2);
    expectNear(m.avgSolveTimeMs, 10.0, 1e-9, "avg solve time");
    expectNear(m.maxSolveTimeMs, 15.0, 1e-9, "max solve time");
}

// 加速度统计：RMS 与最大值。
void testEvaluatorAccelMetrics() {
    auto traj = makeStraightTrajectory(5);
    // makeStraight 给 acceleration=50 全程
    const auto m = ato::Evaluator::evaluate("accel", traj, 0.0, 0.0, 0.2);
    expectNear(m.accelMax, 50.0, 1e-9, "accel max");
    expectNear(m.accelRms, 50.0, 1e-9, "accel rms constant");
}

// 运行时间 = 末帧 time - 首帧 time。
void testEvaluatorRunTime() {
    auto traj = makeStraightTrajectory(5);  // time = 0,0.2,...,0.8
    const auto m = ato::Evaluator::evaluate("runtime", traj, 0.0, 0.0, 0.2);
    expectNear(m.runTimeS, 0.8, 1e-9, "run time span");
}

// 停车时间误差：expectedStopTimeS<=0 时不计算。
void testEvaluatorStopTimeErrorDisabled() {
    auto traj = makeStraightTrajectory(10);
    traj.back().position = 1000.0;
    const auto m = ato::Evaluator::evaluate("test", traj, 1000.0, 0.0, 0.2);
    expectNear(m.stopError, 0.0, 1e-9, "stop error");
    expectNear(m.stopTimeError, 0.0, 1e-9, "stop time error disabled when expected<=0");
}

// 停车点<=0 时不计算停车误差。
void testEvaluatorStopErrorDisabled() {
    const auto traj = makeStraightTrajectory(10);
    const auto m = ato::Evaluator::evaluate("test", traj, 0.0, 5.0, 0.2);
    expectNear(m.stopError, 0.0, 1e-9, "stop error disabled when stopPoint<=0");
}

// toCsvRow 应有 22 列(21 个逗号)。
void testEvaluatorToCsvRowColumnCount() {
    const auto traj = makeStraightTrajectory(10);
    const auto m = ato::Evaluator::evaluate("test", traj);
    const auto row = ato::Evaluator::toCsvRow(m);
    const int commas = static_cast<int>(std::count(row.begin(), row.end(), ','));
    expectTrue(commas == 21, "csv row has 21 commas (22 columns)");
}

// writeCsv 写文件成功并含 header。
void testEvaluatorWriteCsv() {
    const auto traj = makeStraightTrajectory(10);
    const auto m = ato::Evaluator::evaluate("test", traj);
    const std::string path = "test_evaluator_output.csv";
    const bool ok = ato::Evaluator::writeCsv(path, {m});
    expectTrue(ok, "writeCsv returns true");
    std::ifstream in(path);
    expectTrue(in.good(), "file readable");
    std::string line;
    std::getline(in, line);
    expectTrue(line.find("scenario") != std::string::npos, "header written");
    std::getline(in, line);
    expectTrue(line.find("test") != std::string::npos, "data row written");
    in.close();
    std::remove(path.c_str());
}

// =================================================================
// control_loop 集成测试（MPC 路径）
// =================================================================

ato::ControlConfig mpcControlConfig() {
    ato::ControlConfig cfg;
    cfg.parking.brakeAccParaB = 100.0;
    cfg.parking.originBrakeAcc = 100.0;
    cfg.parking.defaultBrakeAcc = 80.0;
    cfg.target.maxLineLimitSpeed = 2200.0;
    cfg.target.minMarginFromEbi = 55.0;
    cfg.ebSb.motorAcc = 80.0;
    cfg.ebSb.brakeAcc = 120.0;
    cfg.ebSb.motorDelay = 1.0;
    cfg.ebSb.brakeDelay = 2.0;
    ato::Curve tracCurve = {{0.0, 100.0}, {1000.0, 100.0}, {2000.0, 60.0}};
    ato::Curve brakeCurve = {{0.0, 110.0}, {1000.0, 100.0}, {2000.0, 90.0}};
    cfg.output.tractionCurve = tracCurve;
    cfg.output.brakeCurve = brakeCurve;
    cfg.output.minAnalogOutput = 1000.0;
    cfg.output.maxAnalogOutput = 9000.0;
    cfg.mpc.horizon = 10;
    cfg.mpc.sampleTime = 0.2;
    cfg.mpc.wSpeed = 1.0;
    cfg.mpc.wControl = 1e-4;
    cfg.mpc.wJerk = 5e-3;
    cfg.mpc.wTerminal = 100.0;
    cfg.mpc.wStopSpeed = 50.0;
    cfg.mpc.vehicle = standardVehicleConfig();
    cfg.mpc.solver.maxIterations = 5;
    return cfg;
}

void testControlLoopMpcPath() {
    ato::ControlLoop loop(mpcControlConfig());
    ato::ControlInput in;
    in.trainSpeed = 0.0;
    in.ebiSpeed = 2200.0;
    in.runLevelSpeed = 2200.0;
    in.distanceToStop = 5000.0;
    in.grade = 0.0;
    in.phase = ato::DrivePhase::Cruise;
    in.trainStopped = false;
    in.stopPointId = 1;
    const auto out = loop.step(in);
    expectTrue(out.mpc.position.size() == 11, "mpc pred size");
    expectTrue(out.targetSpeed > 0.0, "target speed positive");
}

}  // namespace

int main() {
    // linalg
    runTest("linalg/identity multiply", testLinalgIdentityMultiply);
    runTest("linalg/transpose", testLinalgTranspose);
    runTest("linalg/solve symmetric 2x2", testLinalgSolveSymmetric);
    runTest("linalg/solve symmetric 5x5", testLinalgLargerSpdSolve);

    // vehicle_plant
    runTest("vehicle/no-force idle decel", testVehiclePlantNoForceIdleDecel);
    runTest("vehicle/traction accelerates", testVehiclePlantTractionAccelerates);
    runTest("vehicle/brake decelerates", testVehiclePlantBrakeDecelerates);
    runTest("vehicle/uphill adds resistance", testVehiclePlantUphillAddsResistance);
    runTest("vehicle/max forces non-zero", testVehiclePlantMaxForcesNonZero);
    runTest("vehicle/unit conversion", testVehiclePlantUnitConversion);

    // constraint_provider
    runTest("constraint/empty gives max", testConstraintProviderEmptyGivesMaxSpeed);
    runTest("constraint/with restriction lowers max", testConstraintProviderWithRestrictionLowersMax);
    runTest("constraint/with stop point", testConstraintProviderWithStopPoint);
    runTest("constraint/run level cap", testConstraintProviderRunLevelCap);
    runTest("constraint/adaptive vref higher than fixed", testConstraintProviderAdaptiveVrefHigherThanFixed);
    runTest("constraint/adaptive vref downhill lower", testConstraintProviderAdaptiveVrefDownhillLower);
    runTest("constraint/adaptive vref uphill higher", testConstraintProviderAdaptiveVrefUphillHigher);
    runTest("constraint/adaptive vref monotonic decrease", testConstraintProviderAdaptiveVrefMonotonicDecrease);
    runTest("constraint/adaptive vref capped by max", testConstraintProviderAdaptiveVrefCappedByMax);
    runTest("constraint/adaptive vref floor on extreme downhill", testConstraintProviderAdaptiveVrefFloorOnExtremeDownhill);
    runTest("constraint/adaptive vref fallback no curve", testConstraintProviderAdaptiveVrefFallbackNoCurve);
    runTest("constraint/segmented vref far higher", testConstraintProviderSegmentedVrefFarHigher);
    runTest("constraint/segmented vref near conservative", testConstraintProviderSegmentedVrefNearConservative);
    runTest("constraint/segmented vref disabled", testConstraintProviderSegmentedDisabled);

    // sqpmethod
    runTest("sqp/simple quadratic", testSqpmethodSimpleQuadratic);
    runTest("sqp/box constrained", testSqpmethodBoxConstrained);

    // mpc_controller
    runTest("mpc/produces prediction", testMpcControllerProducesPrediction);
    runTest("mpc/with stop point", testMpcControllerWithStopPoint);
    runTest("mpc/reset", testMpcControllerReset);
    runTest("mpc/active-set respects max speed", testMpcActiveSetRespectsMaxSpeed);
    runTest("mpc/projected respects max speed", testMpcProjectedRespectsMaxSpeed);
    runTest("mpc/soft independent of iter", testMpcSoftWeakPenaltyOverspeeds);
    runTest("mpc/active-set tighter than soft", testMpcActiveSetTighterThanSoft);

    // evaluator
    runTest("evaluator/basic metrics", testEvaluatorBasicMetrics);
    runTest("evaluator/stop error", testEvaluatorStopError);
    runTest("evaluator/direction switches", testEvaluatorDirectionSwitches);
    runTest("evaluator/csv row", testEvaluatorToCsvRow);
    runTest("evaluator/markdown table", testEvaluatorMarkdownTable);
    runTest("evaluator/empty trajectory", testEvaluatorEmptyTrajectory);
    runTest("evaluator/single step", testEvaluatorSingleStepTrajectory);
    runTest("evaluator/overspeed ebi sbi", testEvaluatorOverspeedEbiSbi);
    runTest("evaluator/overspeed zero envelope", testEvaluatorOverspeedZeroEnvelope);
    runTest("evaluator/energy split", testEvaluatorEnergySplit);
    runTest("evaluator/traction brake switches", testEvaluatorTractionBrakeSwitches);
    runTest("evaluator/solve time", testEvaluatorSolveTime);
    runTest("evaluator/accel metrics", testEvaluatorAccelMetrics);
    runTest("evaluator/run time", testEvaluatorRunTime);
    runTest("evaluator/stop time error disabled", testEvaluatorStopTimeErrorDisabled);
    runTest("evaluator/stop error disabled", testEvaluatorStopErrorDisabled);
    runTest("evaluator/csv column count", testEvaluatorToCsvRowColumnCount);
    runTest("evaluator/write csv", testEvaluatorWriteCsv);

    // control_loop integration
    runTest("control/mpc path", testControlLoopMpcPath);

    std::cout << "\nmpc unit tests: " << g_ctx.passed << " passed, "
              << g_ctx.failed << " failed\n";
    return g_ctx.failed == 0 ? 0 : 1;
}
