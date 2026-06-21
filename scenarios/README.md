# ATO 仿真场景库

本目录包含用于 PID vs MPC 对比评估的典型场景 CSV。

## 场景列表

| 场景 | 文件 | 重点验证 |
|------|------|----------|
| 正常巡航 | `normal_cruise.csv` | 速度跟踪 RMSE |
| 限速逼近 | `speed_restriction_approach.csv` | MPC 是否提前减速 |
| 精确停车-平道 | `precise_stop_normal.csv` | 停车误差 |
| 精确停车-上坡 | `precise_stop_uphill.csv` | 坡道停车（最容易过冲） |
| 精确停车-下坡 | `precise_stop_downhill.csv` | 坡道停车（最容易欠停） |
| 雨雪低黏着 | `rain_low_adhesion.csv` | Jerk 表现 |
| 综合全线路 | `full_line_simulation.csv` | 多限速 + 停车点 |

## CSV 列定义

```
0  time_s
1  train_speed
2  ebi_speed
3  run_level_speed
4  distance_to_stop
5  grade
6  phase (cruise|stop|0|1)
7  train_stopped
8  stop_point_id
9  restriction_distance     多限速用 ; 分隔
10 restriction_speed       多限速用 ; 分隔
11 restriction_grade       多限速用 ; 分隔
```

## 运行

```bash
# PID 路径
./ato_sim_cli --controller=pid scenarios/precise_stop_normal.csv

# MPC 路径
./ato_sim_cli --controller=mpc scenarios/precise_stop_normal.csv

# 带指标评估
./ato_sim_cli --controller=mpc --evaluate scenarios/precise_stop_normal.csv
```
