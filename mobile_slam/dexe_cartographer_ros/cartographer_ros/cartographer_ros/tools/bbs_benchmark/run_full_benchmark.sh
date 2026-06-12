#!/bin/bash
# =============================================================================
# BBS 全局定位基准——全版本 × 全场景一键测试（NX / x86 通用）
#
# 跑完 MATCHERS 中每个版本的 4 个场景（小图 synth / 小图 bag / 大图 synth /
# 大图 bag），全部报告收进一个带时间戳的目录并生成 summary.txt 汇总，最后
# 打包成 tar.gz 方便回传分析。
#
# 用法:
#   ./run_full_benchmark.sh                # 全默认
#   WS=/path/to/ws ./run_full_benchmark.sh # 覆盖工作区路径
#   MATCHERS="v0 v1.1" ./run_full_benchmark.sh   # 只测部分版本
#
# NX 注意：先开最大性能模式，否则数据不可比——
#   sudo nvpmodel -m 0 && sudo jetson_clocks
# =============================================================================
# 注意不要用 set -u：ROS 的 setup.bash 里有未绑定变量

# ---------------- 配置区（按机器实际情况修改） ----------------
WS="${WS:-$HOME/dexforce/0_slam/acc_global_localization/dexe_mobile_slam}"
MATCHERS="${MATCHERS:-v0 v1 v1.1 v2.1}"   # 可加 v2；无 GPU 的机器去掉 v2/v2.1
SMALL_MAP="${SMALL_MAP:-offline_map}"
BIG_MAP="${BIG_MAP:-offline_map_big}"
SMALL_BAG="${SMALL_BAG:-$HOME/datasets/mapping}"
BIG_BAG="${BIG_BAG:-$HOME/datasets/dongguan/bags/mapping_big_filtered}"
NUM_POSES="${NUM_POSES:-30}"              # synth 位姿数/场景
BAG_SAMPLES="${BAG_SAMPLES:-30}"          # bag 随机抽帧数/场景
SEED="${SEED:-42}"                        # 所有版本同 seed 保证可比
# ---------------------------------------------------------------

STAMP=$(date +%Y%m%d_%H%M%S)
HOST=$(hostname)
OUT="$HOME/.dexe/localization/bbs_baseline/full_${HOST}_${STAMP}"
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

run_one() {  # $1=matcher $2=map $3=mode $4=bag(可空)
  local tag="$1_$2_$3"
  echo "==== [$(date +%H:%M:%S)] $tag ===="
  if [ "$3" = "synth" ]; then
    ros2 run cartographer_ros bbs_benchmark_tool \
      --map_name "$2" --mode synth --num_poses "$NUM_POSES" \
      --matcher "$1" --seed "$SEED" --pyramid_rebuilds 1 \
      --output_dir "$OUT" > "$OUT/log_$tag.txt" 2>&1
  else
    ros2 run cartographer_ros bbs_benchmark_tool \
      --map_name "$2" --mode bag --bag_filename "$4" \
      --bag_random_samples "$BAG_SAMPLES" \
      --matcher "$1" --seed "$SEED" --pyramid_rebuilds 1 \
      --output_dir "$OUT" > "$OUT/log_$tag.txt" 2>&1
  fi
  if [ $? -ne 0 ]; then
    echo "  ❌ 失败（详见 log_$tag.txt）——继续后续测试"
    echo "$tag: FAILED" >> "$OUT/summary.txt"
  else
    grep -E "总耗时" "$OUT/log_$tag.txt" | head -1 | sed "s/^/  /"
  fi
}

echo "输出目录: $OUT"
{
  echo "===== BBS 全版本基准 ====="
  echo "host: $HOST | date: $STAMP | seed: $SEED"
  echo "versions: $MATCHERS | poses: $NUM_POSES | bag_samples: $BAG_SAMPLES"
  uname -m
  nvidia-smi -L 2>/dev/null || echo "(无独立 GPU / Jetson 平台)"
  command -v nvpmodel >/dev/null && nvpmodel -q 2>/dev/null | head -2
  echo ""
} | tee "$OUT/summary.txt"

for m in $MATCHERS; do
  run_one "$m" "$SMALL_MAP" synth
  run_one "$m" "$BIG_MAP"   synth
  run_one "$m" "$SMALL_MAP" bag "$SMALL_BAG"
  run_one "$m" "$BIG_MAP"   bag "$BIG_BAG"
done

# 汇总：每份报告抽取关键行（matcher/地图/耗时/匹配率/成功率/误差/金字塔）
{
  echo ""
  echo "===== 汇总（mean/median/p95 自动提取）====="
  for f in "$OUT"/*_synth.txt "$OUT"/*_bag.txt; do
    [ -f "$f" ] || continue
    echo "--- $(basename "$f")"
    grep -E "matcher:|地图:|金字塔构建（最后|总耗时|匹配率|成功率|样本误差" "$f"
  done
} >> "$OUT/summary.txt"

tar -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
echo ""
echo "✅ 全部完成"
echo "   汇总: $OUT/summary.txt"
echo "   打包: $OUT.tar.gz   ← 把这个文件（或 summary.txt）发回分析"
