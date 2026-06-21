#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 ATO MPC 分析面板 HTML。

跑多组 MPC 配置(Soft/Projected/ActiveSet × 分段vref)收集轨迹 CSV，
生成自包含 HTML：曲线对比图(速度/力/目标)+指标对比表。
PID 对比数据见 README 静态表格（本仿真包不再包含 PID 运行时）。
用法: python3 tools/gen_analysis_html.py
产物: ato_analysis.html
"""
import csv
import io
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "build", "ato_sim_cli")

# 配置矩阵: (标签, constraint, vref-margin, vref-margin-far)
CONFIGS = [
    ("MPC-Soft",      "soft",      None, None),
    ("MPC-B1投影",    "projected", None, None),
    ("MPC-B2保守",    "activeset", "0.4", None),
    ("MPC-B2分段",    "activeset", "0.4", "0.8"),
]

# 场景: (文件, 标签, 是否停车场景)
SCENARIOS = [
    ("scenarios/realistic_station_1km.csv",          "1km 平坡",        True),
    ("scenarios/realistic_station_2km_uphill.csv",   "2km 上坡",        True),
    ("scenarios/realistic_line_1500m_restricted.csv","1.5km 限速",      True),
    ("examples/stop_case.csv",                       "stop_case",       True),
    ("scenarios/precise_stop_uphill.csv",            "精确停车-上坡",   True),
    ("scenarios/precise_stop_downhill.csv",          "精确停车-下坡",   True),
]

CSV_COLS = ["time_s","train_speed","target_speed","cur_max_sbv","parking_brake_acc",
            "pid_demand","mpc_force","mpc_solve_ms","direction","analog_output",
            "traction","brake","brake_hold","stop_adjust","state"]


def run_cfg(scenario, cfg):
    """跑一个配置，返回 (rows, metrics)。"""
    label, cmode, margin, margin_far = cfg
    args = [CLI, "--evaluate", scenario]
    if cmode: args.insert(1, f"--constraint={cmode}")
    if margin: args.insert(1, f"--vref-margin={margin}")
    if margin_far:
        args.insert(1, f"--vref-margin-far={margin_far}")
        args.insert(1, "--vref-far-dist=30000")
    # 停车场景不需要 duration；巡航场景给足时长
    if not any(s[0] == scenario for s in SCENARIOS if s[2]):
        args.insert(1, "--duration=90")
    out = subprocess.run(args, capture_output=True, text=True, cwd=ROOT)
    rows = []
    metrics = {}
    in_eval = False
    for line in out.stdout.splitlines():
        if line.startswith("# Evaluation"):
            in_eval = True
            continue
        if in_eval:
            if line.startswith("# ") and "scenario," in line:
                continue
            if line.startswith("# "):
                parts = line[2:].split(",")
                keys = ["scenario","stop_err","stop_time_err","speed_rmse","speed_max_err",
                        "jerk_rms","jerk_max","dir_sw","energy","avg_solve","max_solve","steps",
                        "run_time","trac_energy","brake_energy","over_ebi","over_ebi_max",
                        "over_sbi","over_sbi_max","accel_rms","accel_max","trac_brake_sw"]
                for i, k in enumerate(keys):
                    if i < len(parts):
                        try: metrics[k] = float(parts[i])
                        except ValueError: metrics[k] = parts[i]
                in_eval = False
            continue
        if line.startswith("time_s") or not line.strip():
            continue
        vals = line.split(",")
        if len(vals) >= len(CSV_COLS):
            r = {}
            for i, c in enumerate(CSV_COLS):
                try: r[c] = float(vals[i])
                except ValueError: r[c] = vals[i]
            rows.append(r)
    return rows, metrics


def collect():
    data = {"scenarios": []}
    for scen_path, scen_label, is_stop in SCENARIOS:
        scen = {"name": scen_label, "path": scen_path, "configs": []}
        for cfg in CONFIGS:
            label, cmode, margin, margin_far = cfg
            rows, metrics = run_cfg(scen_path, cfg)
            scen["configs"].append({
                "label": label,
                "constraint": cmode or "soft",
                "rows": rows,
                "metrics": metrics,
            })
        data["scenarios"].append(scen)
    return data


HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>ATO 控制算法分析面板</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  body { font-family: -apple-system, "PingFang SC", sans-serif; margin: 16px; background: #f7f8fa; color: #222; }
  h1 { font-size: 20px; margin: 0 0 4px; }
  h2 { font-size: 16px; margin: 18px 0 8px; border-bottom: 2px solid #4a90d9; padding-bottom: 4px; }
  .desc { color: #666; font-size: 13px; margin-bottom: 12px; }
  .ctrl { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-bottom: 12px; }
  .ctrl label { font-size: 13px; }
  select, button { padding: 4px 8px; font-size: 13px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .card { background: #fff; border: 1px solid #e0e3e8; border-radius: 6px; padding: 10px; }
  .card h3 { font-size: 13px; margin: 0 0 8px; color: #4a90d9; }
  canvas { max-height: 260px; }
  table { border-collapse: collapse; width: 100%; font-size: 12px; }
  th, td { border: 1px solid #e0e3e8; padding: 4px 6px; text-align: right; }
  th { background: #eef2f7; font-weight: 600; }
  td:first-child, th:first-child { text-align: left; }
  .best { background: #e6f4ea; font-weight: 600; }
  .warn { color: #c0392b; }
  .legend { font-size: 11px; color: #888; margin-top: 4px; }
</style></head><body>
<h1>ATO MPC 控制算法分析面板 (Soft/B1/B2/分段 vref)</h1>
<div class="desc">真实尺度场景闭环仿真。曲线: 速度/目标速度、牵引制动力、加速度。指标表: 停车精度/运行时间/能耗/安全/舒适。</div>

<div class="ctrl">
  <label>场景:</label>
  <select id="scenSel"></select>
  <label>曲线视图:</label>
  <select id="viewSel">
    <option value="speed">速度跟踪 (列车/目标)</option>
    <option value="force">牵引/制动力</option>
    <option value="accel">加速度</option>
    <option value="target_curve">目标速度包络</option>
  </select>
</div>

<h2>曲线对比</h2>
<div class="card"><canvas id="chartCurve"></canvas>
  <div class="legend">各配置同图叠加；hover 看数值。停车场景横轴为运行时间，纵轴单位见视图。</div>
</div>

<h2>指标对比表</h2>
<div class="card">
  <table id="metricTbl"></table>
</div>

<h2>说明</h2>
<div class="desc">
  • <b>MPC-Soft</b>: 软约束 • <b>MPC-B1投影</b>: 投影法硬约束 • <b>MPC-B2保守</b>: 主动集硬约束+vref保守 • <b>MPC-B2分段</b>: 主动集+分段vref(远激进近保守)<br>
  • 绿色行=该场景停车误差最小。超 EBI 应为 0(安全合规)。<br>
  • 数据由 <code>tools/gen_analysis_html.py</code> 跑当前 CLI 生成，自包含无外部依赖(除 Chart.js CDN)。
</div>

<script>
const DATA = __DATA__;

const COLORS = {"MPC-Soft":"#e67e22","MPC-B1投影":"#9b59b6","MPC-B2保守":"#2980b9","MPC-B2分段":"#27ae60"};
let chart = null;

function fillSelects() {
  const sel = document.getElementById("scenSel");
  DATA.scenarios.forEach((s, i) => {
    const o = document.createElement("option");
    o.value = i; o.textContent = s.name;
    sel.appendChild(o);
  });
}

function buildCurve(scenIdx, view) {
  const scen = DATA.scenarios[scenIdx];
  const datasets = [];
  scen.configs.forEach(c => {
    const rows = c.rows;
    let label, field, yLabel;
    if (view === "speed") {
      // 两条: 列车速度 + 目标速度(仅对第一个有目标的)
      datasets.push({
        label: c.label + " 列车",
        data: rows.map(r => ({x: r.time_s, y: r.train_speed})),
        borderColor: COLORS[c.label],
        backgroundColor: COLORS[c.label],
        pointRadius: 0, borderWidth: 2, tension: 0.1
      });
      if (c.label === "MPC-Soft") {
        datasets.push({
          label: "目标速度",
          data: rows.map(r => ({x: r.time_s, y: r.target_speed})),
          borderColor: "#c0392b", borderDash: [4,3],
          pointRadius: 0, borderWidth: 1.5, tension: 0.1
        });
      }
    } else if (view === "force") {
      datasets.push({
        label: c.label,
        data: rows.map(r => ({x: r.time_s, y: r.mpc_force})),
        borderColor: COLORS[c.label], pointRadius: 0, borderWidth: 2, tension: 0.1
      });
    } else if (view === "accel") {
      // 估算加速度: 速度差/周期
      const acc = rows.map((r, i) => i === 0 ? 0 :
        (r.train_speed - rows[i-1].train_speed) / 0.2);
      datasets.push({
        label: c.label,
        data: rows.map((r, i) => ({x: r.time_s, y: acc[i]})),
        borderColor: COLORS[c.label], pointRadius: 0, borderWidth: 2, tension: 0.1
      });
    } else if (view === "target_curve") {
      datasets.push({
        label: c.label + " 目标",
        data: rows.map(r => ({x: r.time_s, y: r.target_speed})),
        borderColor: COLORS[c.label], pointRadius: 0, borderWidth: 2, tension: 0.1
      });
    }
  });
  return datasets;
}

function render() {
  const scenIdx = +document.getElementById("scenSel").value;
  const view = document.getElementById("viewSel").value;
  const scen = DATA.scenarios[scenIdx];
  const datasets = buildCurve(scenIdx, view);
  const yLabel = {"speed":"速度 (cm/s)","force":"力 (N)","accel":"加速度 (cm/s^2)","target_curve":"目标速度 (cm/s)"}[view];

  if (chart) chart.destroy();
  const ctx = document.getElementById("chartCurve").getContext("2d");
  chart = new Chart(ctx, {
    type: "line",
    data: {datasets},
    options: {
      responsive: true,
      interaction: {mode: "nearest", intersect: false, axis: "x"},
      scales: {
        x: {type: "linear", title: {display: true, text: "时间 (s)"}},
        y: {title: {display: true, text: yLabel}}
      },
      plugins: {tooltip: {mode: "nearest", intersect: false}}
    }
  });
  renderTable(scenIdx);
}

function renderTable(scenIdx) {
  const scen = DATA.scenarios[scenIdx];
  const cols = [
    ["配置","label"],["停车cm","stop_err"],["运行s","run_time"],
    ["速度RMSE","speed_rmse"],["加速RMS","accel_rms"],["JerkRMS","jerk_rms"],
    ["总能耗kJ","energy_kj"],["再生kJ","brake_energy_kj"],["超EBI","over_ebi"],
    ["方向切","dir_sw"],["求解ms","avg_solve"]
  ];
  // 找最小停车误差
  let bestStop = Infinity;
  scen.configs.forEach(c => {
    const m = c.metrics;
    const e = m.stop_err || 0;
    if (e < bestStop) bestStop = e;
  });
  let html = "<thead><tr>" + cols.map(c => `<th>${c[0]}</th>`).join("") + "</tr></thead><tbody>";
  scen.configs.forEach(c => {
    const m = c.metrics || {};
    const stopErr = m.stop_err || 0;
    const isBest = Math.abs(stopErr - bestStop) < 0.5;
    const energyKj = (m.energy || 0) / 1000;
    const brakeKj = (m.brake_energy || 0) / 1000;
    const overEbi = m.over_ebi || 0;
    const row = [
      c.label, stopErr.toFixed(0), (m.run_time||0).toFixed(1),
      (m.speed_rmse||0).toFixed(0), (m.accel_rms||0).toFixed(0), (m.jerk_rms||0).toFixed(0),
      energyKj.toFixed(0), brakeKj.toFixed(0),
      `<span class="${overEbi>0?'warn':''}">${overEbi}</span>`,
      m.dir_sw||0, (m.avg_solve||0).toFixed(2)
    ];
    html += `<tr class="${isBest?'best':''}">` + row.map((v,i) => `<td>${v}</td>`).join("") + "</tr>";
  });
  html += "</tbody>";
  document.getElementById("metricTbl").innerHTML = html;
}

fillSelects();
document.getElementById("scenSel").onchange = render;
document.getElementById("viewSel").onchange = render;
render();
</script>
</body></html>
"""


def main():
    if not os.path.exists(CLI):
        print(f"ERROR: CLI 不存在: {CLI}", file=sys.stderr)
        print("请先 cmake --build build", file=sys.stderr)
        return 1
    print("收集数据中(跑配置矩阵)...")
    data = collect()
    html = HTML_TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False))
    out_path = os.path.join(ROOT, "ato_analysis.html")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"已生成: {out_path}")
    print(f"  {len(data['scenarios'])} 场景 × {len(CONFIGS)} 配置")
    return 0


if __name__ == "__main__":
    sys.exit(main())
