#!/bin/bash
# =============================================================================
# PoseInitializer 端到端全局重定位基准——全版本 × 全地图一键测试（x86 / NX 通用）
#
# 测的是整条全局重定位流水线（global matching → online correlative refine →
# line tracking → ceres refine → validation）的端到端耗时，而非单独的全图匹配
# 阶段（后者用 run_full_benchmark.sh / bbs_benchmark_tool）。
#
# 与 run_full_benchmark.sh 的区别：三版本对比（original/cpu/gpu）在工具内由
# --matcher_types 一次跑完，故本脚本只循环 地图 × 模式（每张图 synth + bag）。
# 每次调用产出 3 个报告（每版本一个），且横向对比表写进每个报告。
#
# 用法:
#   ./run_full_pose_initializer_benchmark.sh                       # 全默认
#   WS=/path/to/ws ./run_full_pose_initializer_benchmark.sh        # 覆盖工作区
#   MAPS="offline_map" ./run_full_pose_initializer_benchmark.sh    # 只测一张图
#   MATCHER_TYPES="cpu,gpu" ./run_full_pose_initializer_benchmark.sh  # 只测部分版本
#
# NX 注意：先开最大性能模式，否则数据不可比——
#   sudo nvpmodel -m 0 && sudo jetson_clocks
# =============================================================================
# 注意不要用 set -u：ROS 的 setup.bash 里有未绑定变量

# ---------------- 配置区（按机器实际情况修改） ----------------
WS="${WS:-$HOME/dexforce/0_slam/acc_global_localization/dexe_mobile_slam}"
MATCHER_TYPES="${MATCHER_TYPES:-original,cpu,gpu}"  # 工具内一次跑全部；无 GPU 自动跳过
SMALL_MAP="${SMALL_MAP:-offline_map_small_2}"
BIG_MAP="${BIG_MAP:-offline_map_big_2}"
SMALL_BAG="${SMALL_BAG:-$HOME/datasets/mapping}"
BIG_BAG="${BIG_BAG:-$HOME/datasets/dongguan/bags/mapping_big_filtered}"
MAPS="${MAPS:-$SMALL_MAP $BIG_MAP}"       # 要测的地图列表（空格分隔）
MODES="${MODES:-synth bag}"               # synth / bag / 两者
NUM_POSES="${NUM_POSES:-30}"              # synth 位姿数/场景
BAG_SAMPLES="${BAG_SAMPLES:-30}"          # bag 随机抽帧数/场景
SEED="${SEED:-42}"                        # 所有版本同 seed 保证可比
# ---------------------------------------------------------------

STAMP=$(date +%Y%m%d_%H%M%S)
HOST=$(hostname)
OUT="$HOME/.dexe/localization/poseinit_baseline/full_${HOST}_${STAMP}"
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 地图 → bag 路径映射
bag_for() {
  case "$1" in
    "$SMALL_MAP") echo "$SMALL_BAG" ;;
    "$BIG_MAP")   echo "$BIG_BAG" ;;
    *)            echo "" ;;
  esac
}

run_one() {  # $1=map $2=mode
  local map="$1" mode="$2"
  local tag="${map}_${mode}"
  local log="$OUT/log_${tag}.txt"
  echo "==== [$(date +%H:%M:%S)] $tag （版本: $MATCHER_TYPES）===="
  if [ "$mode" = "synth" ]; then
    ros2 run cartographer_ros pose_initializer_benchmark_tool \
      --map_name "$map" --mode synth --num_poses "$NUM_POSES" \
      --matcher_types "$MATCHER_TYPES" --seed "$SEED" \
      --output_dir "$OUT" > "$log" 2>&1
  else
    local bag; bag="$(bag_for "$map")"
    if [ -z "$bag" ] || [ ! -e "$bag" ]; then
      echo "  ⏭  跳过（无 bag：$bag）"
      echo "$tag: SKIPPED (no bag)" >> "$OUT/summary.txt"
      return
    fi
    ros2 run cartographer_ros pose_initializer_benchmark_tool \
      --map_name "$map" --mode bag --bag_filename "$bag" \
      --bag_random_samples "$BAG_SAMPLES" \
      --matcher_types "$MATCHER_TYPES" --seed "$SEED" \
      --output_dir "$OUT" > "$log" 2>&1
  fi
  if [ $? -ne 0 ]; then
    echo "  ❌ 失败（详见 log_$tag.txt）——继续后续测试"
    echo "$tag: FAILED" >> "$OUT/summary.txt"
  else
    # 打印该场景的三版本横向对比表（端到端总耗时）
    grep -A4 "版本横向对比" "$log" | sed "s/^/  /"
  fi
}

echo "输出目录: $OUT"
{
  echo "===== PoseInitializer 端到端全版本基准 ====="
  echo "host: $HOST | date: $STAMP | seed: $SEED"
  echo "maps: $MAPS | modes: $MODES | types: $MATCHER_TYPES"
  echo "poses: $NUM_POSES | bag_samples: $BAG_SAMPLES"
  uname -m
  nvidia-smi -L 2>/dev/null || echo "(无独立 GPU / Jetson 平台)"
  command -v nvpmodel >/dev/null && nvpmodel -q 2>/dev/null | head -2
  echo ""
} | tee "$OUT/summary.txt"

for map in $MAPS; do
  for mode in $MODES; do
    run_one "$map" "$mode"
  done
done

# 汇总：每个场景的三版本横向对比 + 各版本端到端耗时/阶段占比/成功率
{
  echo ""
  echo "===== 汇总（端到端总耗时 + 阶段占比 + 匹配/成功率）====="
  for log in "$OUT"/log_*.txt; do
    [ -f "$log" ] || continue
    echo ""
    echo "--- $(basename "$log" .txt | sed 's/^log_//')"
    grep -A4 "版本横向对比" "$log"
    grep -E "端到端汇总|全局重定位总耗时|BBS 全图匹配|Ceres 精匹配|匹配率|成功率" "$log"
  done
} >> "$OUT/summary.txt"

tar -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
echo ""
echo "✅ 全部完成"
echo "   汇总: $OUT/summary.txt"
echo "   打包: $OUT.tar.gz   ← 把这个文件（或 summary.txt）发回分析"
