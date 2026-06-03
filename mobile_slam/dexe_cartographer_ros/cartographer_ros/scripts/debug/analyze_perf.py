#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chunk map A/B 性能分析：解析 [PERF] 运行时间日志 + 资源监控 CSV，输出表格。

运行时间来源：定位/离线节点日志里的 `[PERF]` 行（LocalMapUpdater2D / ChunkMap2D 埋点，
周期性 + 退出时各 Dump 一次；累积统计，取最后一次最完整）。
CPU/内存来源：monitor_cartographer_resources.sh 落的 localization_resources.csv
（列：timestamp,time,cpu_percent,rss_mb,mem_percent）。注意 ps 的 %cpu 是进程**生命周期均值**，
反映趋势而非瞬时；峰值列可作粗略上界参考。

用法：
  # 单组：打印该组运行时间 + CPU 表
  analyze_perf.py --label "大图/on" --perf run_big_on.log --cpu big_on_resources.csv

  # 多组对比（重复传 --group 名:perf:cpu）：输出对比表 + A/B 倍数
  analyze_perf.py \
     --group "小图/off:small_off.log:small_off.csv" \
     --group "小图/on:small_on.log:small_on.csv" \
     --group "大图/off:big_off.log:big_off.csv" \
     --group "大图/on:big_on.log:big_on.csv"
"""
import argparse
import csv
import re
import sys


def _is_float(tok):
    try:
        float(tok)
        return True
    except ValueError:
        return False


def parse_perf_log(path):
    """解析一个日志文件里的所有 [PERF] 行，返回 (timings, scalars)。
    timings: {name: {'n','min','mean','p50','p99','max'}}（last-wins，取最后/最完整一次）
    scalars: {name: value}
    """
    timings, scalars = {}, {}
    try:
        with open(path, "r", errors="ignore") as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"[warn] perf 日志不存在: {path}", file=sys.stderr)
        return timings, scalars

    for raw in lines:
        if "[PERF]" not in raw:
            continue
        body = raw.split("[PERF]", 1)[1].strip()
        if not body or body[0] in "=-" or body.startswith("功能") or "运行时间统计" in body or "计数" in body:
            continue
        toks = body.split()
        # 去掉行尾的 "(ms)" 之类
        floats = [t for t in toks if _is_float(t)]
        names = [t for t in toks if not _is_float(t)]
        if not names:
            continue
        name = names[0]
        if len(floats) >= 6:  # 计时行：n min mean p50 p99 max
            n, mn, mean, p50, p99, mx = floats[:6]
            timings[name] = {
                "n": int(float(n)), "min": float(mn), "mean": float(mean),
                "p50": float(p50), "p99": float(p99), "max": float(mx),
            }
        elif len(floats) == 1:  # 标量行：name value
            scalars[name] = float(floats[0])
    return timings, scalars


def parse_cpu_csv(path, skip=0):
    """解析 monitor CSV，返回 {'cpu_mean','cpu_peak','rss_mean_mb','rss_peak_mb','n'}。"""
    cpu, rss = [], []
    try:
        with open(path, "r", errors="ignore") as f:
            for i, row in enumerate(csv.DictReader(f)):
                if i < skip:
                    continue
                try:
                    cpu.append(float(row["cpu_percent"]))
                    rss.append(float(row["rss_mb"]))
                except (KeyError, ValueError):
                    continue
    except FileNotFoundError:
        print(f"[warn] CPU csv 不存在: {path}", file=sys.stderr)
        return None
    if not cpu:
        return None
    return {
        "cpu_mean": sum(cpu) / len(cpu), "cpu_peak": max(cpu),
        "rss_mean_mb": sum(rss) / len(rss), "rss_peak_mb": max(rss), "n": len(cpu),
    }


# 关注的核心计时项（按计划 §3.1 优先级排序展示）
KEY_TIMINGS = [
    "BuildLocalSubmap.total",
    "ChunkMap.AssembleLocalSubmap(on)",
    "BoxFilter.assemble(off)",
    "BoxFilter.snapshot_deepcopy(off)",
    "BoxFilter.grid_crop(off)",
    "BoxFilter.cloud_filter(off)",
    "BoxFilter.RebuildIkd+Finish(off)",
    "ChunkMap.RebuildIkd+Finish(on)",
    "ChunkMap.LoadOnPose(recompute)",
]


def print_single(label, timings, scalars, cpu):
    print(f"\n## {label}\n")
    print("### 运行时间（ms）\n")
    print("| 功能 | n | min | mean | p50 | p99 | max |")
    print("|------|---|-----|------|-----|-----|-----|")
    shown = set()
    for name in KEY_TIMINGS:
        if name in timings:
            t = timings[name]
            print(f"| {name} | {t['n']} | {t['min']:.3f} | {t['mean']:.3f} | "
                  f"{t['p50']:.3f} | {t['p99']:.3f} | {t['max']:.3f} |")
            shown.add(name)
    for name, t in timings.items():  # 其余未列出的也补上
        if name not in shown:
            print(f"| {name} | {t['n']} | {t['min']:.3f} | {t['mean']:.3f} | "
                  f"{t['p50']:.3f} | {t['p99']:.3f} | {t['max']:.3f} |")
    if scalars:
        print("\n### 计数 / 频率\n")
        for k, v in scalars.items():
            print(f"- {k} = {v:.4f}")
    if cpu:
        print("\n### CPU / 内存\n")
        print(f"- CPU%(均值/峰值) = {cpu['cpu_mean']:.2f} / {cpu['cpu_peak']:.2f}  "
              f"（ps %cpu 为生命周期均值，趋势参考）")
        print(f"- RSS MB(均值/峰值) = {cpu['rss_mean_mb']:.1f} / {cpu['rss_peak_mb']:.1f}  "
              f"(采样 {cpu['n']} 点)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--perf")
    ap.add_argument("--cpu")
    ap.add_argument("--skip", type=int, default=0, help="CPU csv 跳过前 N 行(预热)")
    ap.add_argument("--group", action="append", default=[],
                    help='多组: "标签:perf日志:cpucsv"，可重复')
    args = ap.parse_args()

    groups = []
    for g in args.group:
        parts = g.split(":")
        lbl = parts[0]
        perf = parts[1] if len(parts) > 1 and parts[1] else None
        cpu = parts[2] if len(parts) > 2 and parts[2] else None
        groups.append((lbl, perf, cpu))
    if args.perf or args.cpu:
        groups.append((args.label, args.perf, args.cpu))

    if not groups:
        ap.print_help()
        return

    parsed = []
    for lbl, perf, cpu in groups:
        t, s = parse_perf_log(perf) if perf else ({}, {})
        c = parse_cpu_csv(cpu, args.skip) if cpu else None
        parsed.append((lbl, t, s, c))
        print_single(lbl, t, s, c)

    # 多组：BuildLocalSubmap.total mean 对比 + CPU 对比
    if len(parsed) > 1:
        print("\n## 横向对比（BuildLocalSubmap.total）\n")
        print("| 组 | mean(ms) | p99(ms) | rebuild_freq(次/m) | CPU%均值 | RSS峰值MB |")
        print("|----|---------|--------|--------------------|---------|----------|")
        for lbl, t, s, c in parsed:
            bt = t.get("BuildLocalSubmap.total", {})
            mean = f"{bt['mean']:.3f}" if bt else "—"
            p99 = f"{bt['p99']:.3f}" if bt else "—"
            freq = s.get("rebuild_count", 0) / s["robot_distance_m"] if s.get("robot_distance_m") else None
            freqs = f"{freq:.4f}" if freq else "—"
            cpus = f"{c['cpu_mean']:.2f}" if c else "—"
            rsss = f"{c['rss_peak_mb']:.1f}" if c else "—"
            print(f"| {lbl} | {mean} | {p99} | {freqs} | {cpus} | {rsss} |")
        print("\n> 关键看：大图 off vs 大图 on 的 mean/CPU 下降倍数；on 大图 vs on 小图是否接近(规模解耦)。")


if __name__ == "__main__":
    main()
