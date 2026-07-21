#!/bin/bash
# bbs3d_reloc_field_test.sh -- BBS3D 全局重定位现场测试脚本（15 点位 × 10 次/点）
#
# 前置条件：lightning 定位节点已运行（ros2 launch lightning demo_loc_online.launch.py ...）
#           livox 驱动已运行（ros2 launch livox_ros_driver2 msg_MID360_launch.py）
#
# 用法：./bbs3d_reloc_field_test.sh
#   每到一个点位按回车，自动执行 10 次重定位测试，统计成功率/得分/耗时
#   输入 q 可提前结束全部测试

# ==================== 配置 ====================
NODE="/localization_3d_manager"
NUM_POINTS=15
NUM_TRIALS=10
TIMEOUT_SEC=120            # 单次重定位最大等待秒数
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_CSV="${SCRIPT_DIR}/bbs3d_reloc_test_$(date +%Y%m%d_%H%M%S).csv"

# ==================== 日志文件 ====================
# glog 输出：INFO 文件 + WARNING 文件（BBS 成功日志在 INFO，失败日志在 WARNING）
GLOG_INFO="/tmp/run_loc_online.INFO"
GLOG_WARN=$(ls -t /tmp/run_loc_online.*.log.WARNING.* 2>/dev/null | head -1)

if [[ ! -f "$GLOG_INFO" ]]; then
    echo "错误：找不到 glog 日志 $GLOG_INFO"
    echo "请确认定位节点（run_loc_online）正在运行"
    exit 1
fi
GLOG_INFO=$(readlink -f "$GLOG_INFO")
echo "INFO 日志：$GLOG_INFO"

# ==================== ROS2 兜底 ====================
command -v ros2 > /dev/null 2>&1 || source /opt/ros/humble/setup.bash

echo "点位,次数,结果,得分,耗时ms,时间戳" > "$OUT_CSV"
echo "CSV 输出：$OUT_CSV"
echo ""

# ==================== 获取最新的 WARNING 日志 ====================
get_warn_log() {
    # reset 后 glog 可能创建新的 WARNING 文件，取最新的
    local w=$(ls -t /tmp/run_loc_online.*.log.WARNING.* 2>/dev/null | head -1)
    echo "$w"
}

# ==================== 提取新增日志行 ====================
get_new_lines() {
    local info_file="$1"
    local warn_file="$2"
    local info_before="$3"
    local warn_before="$4"
    local new_info=""
    local new_warn=""
    [[ -f "$info_file" ]] && new_info=$(tail -n +$((info_before+1)) "$info_file" 2>/dev/null)
    [[ -f "$warn_file" ]] && new_warn=$(tail -n +$((warn_before+1)) "$warn_file" 2>/dev/null)
    echo -e "${new_info}\n${new_warn}"
}

# ==================== 主测试循环 ====================
for point in $(seq 1 $NUM_POINTS); do
    echo ""
    read -p "=== 点位 ${point}/${NUM_POINTS}：机器人到位后回车开始测试（输入 q 提前结束全部测试）：" ans
    [[ "$ans" == "q" ]] && break

    for trial in $(seq 1 $NUM_TRIALS); do
        # 记录测试前行数
        info_before=$(wc -l < "$GLOG_INFO" 2>/dev/null || echo 0)
        warn_file=$(get_warn_log)
        warn_before=0
        [[ -f "$warn_file" ]] && warn_before=$(wc -l < "$warn_file" 2>/dev/null || echo 0)

        echo "  >>> 点位${point} 第${trial}/${NUM_TRIALS}次：触发重定位..."

        # 触发重置（停止+重启定位，会自动触发 BBS）
        ros2 service call ${NODE}/reset_localization std_srvs/srv/Trigger "{}" > /dev/null 2>&1

        result="超时未完成"; score=""; ms=""
        bbs_triggered=0

        for i in $(seq 1 $TIMEOUT_SEC); do
            sleep 1
            # 更新 WARNING 文件路径（reset 可能产生新文件）
            warn_file=$(get_warn_log)
            new_lines=$(get_new_lines "$GLOG_INFO" "$warn_file" "$info_before" "$warn_before")

            # 检测 BBS 触发
            if [[ $bbs_triggered -eq 0 ]]; then
                if echo "$new_lines" | grep -q "\[BBS\] Triggering global relocalization"; then
                    bbs_triggered=1
                fi
            fi

            # 检查结果
            if echo "$new_lines" | grep -q "\[BBS\] Init success after BBS+NDT"; then
                result="成功"
                reloc_line=$(echo "$new_lines" | grep "\[BBS\] Relocalized" | tail -1)
                ms=$(echo "$reloc_line" | grep -oP '(?<=time )[0-9.]+(?= ms)')
                score=$(echo "$reloc_line" | grep -oP '(?<=score )[0-9]+')
                break
            elif echo "$new_lines" | grep -q "\[BBS\] BBS pose found but NDT refinement failed"; then
                result="失败_NDT"
                reloc_line=$(echo "$new_lines" | grep "\[BBS\] Relocalized" | tail -1)
                ms=$(echo "$reloc_line" | grep -oP '(?<=time )[0-9.]+(?= ms)')
                score=$(echo "$reloc_line" | grep -oP '(?<=score )[0-9]+')
                break
            elif echo "$new_lines" | grep -q "\[BBS\] Relocalization failed"; then
                result="失败_BBS"
                fail_line=$(echo "$new_lines" | grep "\[BBS\] Relocalization failed" | tail -1)
                score=$(echo "$fail_line" | grep -oP '(?<=score )[0-9]+')
                break
            elif echo "$new_lines" | grep -q "\[BBS\] Relocalization timed out"; then
                result="超时_BBS"
                timeout_line=$(echo "$new_lines" | grep "\[BBS\] Relocalization timed out" | tail -1)
                ms=$(echo "$timeout_line" | grep -oP '(?<=\()[0-9.]+(?= ms)')
                break
            fi
        done

        echo "      结果=${result}，得分=${score}，耗时=${ms}ms"
        echo "${point},${trial},${result},${score},${ms},$(date '+%Y-%m-%d %H:%M:%S')" >> "$OUT_CSV"
    done
done

# ==================== 汇总统计 ====================
echo ""
echo "========================================"
echo "=== 测试完成，原始数据：$OUT_CSV ==="
echo "========================================"

awk -F',' '
NR>1 {
    total++
    if ($3=="成功") succ++
    by_total[$1]++
    if ($3=="成功") by_succ[$1]++
    if ($4!="") {
        score_sum[$1]+=$4; score_n[$1]++
        if (score_min[$1]=="" || $4<score_min[$1]) score_min[$1]=$4
        if (score_max[$1]=="" || $4>score_max[$1]) score_max[$1]=$4
    }
    if ($5!="") {
        time_sum[$1]+=$5; time_n[$1]++
        if (time_min[$1]=="" || $5<time_min[$1]) time_min[$1]=$5
        if (time_max[$1]=="" || $5>time_max[$1]) time_max[$1]=$5
    }
}
END {
    printf "\n=== 按点位汇总 ===\n"
    printf "%-6s %-8s %-8s %-12s %-10s %-10s %-12s %-10s %-10s\n", \
        "点位", "成功", "总数", "成功率", "得分mean", "得分min", "耗时mean(ms)", "耗时min", "耗时max"
    for (p=1; p<=15; p++) {
        if (by_total[p] > 0) {
            rate = by_succ[p]*100.0/by_total[p]
            sm = (score_n[p]>0) ? score_sum[p]/score_n[p] : 0
            tm = (time_n[p]>0) ? time_sum[p]/time_n[p] : 0
            printf "%-6d %-8d %-8d %-12.1f%% %-10.1f %-10s %-12.1f %-10s %-10s\n", \
                p, by_succ[p], by_total[p], rate, sm, score_min[p], tm, time_min[p], time_max[p]
        }
    }
    printf "\n=== 总计 ===\n"
    printf "总成功率：%d/%d（%.1f%%）\n", succ, total, (total>0 ? succ*100.0/total : 0)
    if (time_n[0] == "") {
        all_time_sum=0; all_time_n=0; all_time_min=""; all_time_max=""
    }
}' "$OUT_CSV"

echo ""
echo "CSV 文件：$OUT_CSV"
