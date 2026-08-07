#!/bin/sh
# 给安装态 robot_localization UKF/EKF 写入 CAP_SYS_NICE 的**单一来源**。
#
# 三个调用方共用本脚本，避免 setcap 逻辑和节点路径散落多处：
#   1. deb postinst      —— 安装完成后首次授权（STRICT=1，失败即让安装非零退出）
#   2. build.sh          —— 源码工作区构建后首次授权（STRICT=1）
#   3. dexe-auto drop-in —— 每次服务启动前重新授权（默认宽松模式，见下）
#
# 之所以每次启动都要重新跑：共享部署树 /home/dexforce/w1 上有 20+ 个兄弟包的 postinst
# 执行 `chown -R dexforce:dexforce /home/dexforce/w1`，而 chown(2) 对非目录无条件清除
# security.capability（chown 到相同 owner 也清）。一次性写入的 capability 守不住。
#
# 用法：ensure_realtime_caps.sh [INSTALL_DIR]
#   INSTALL_DIR 默认 /home/dexforce/w1/install
#
# 环境变量：
#   STRICT=1  任一步失败即非零退出；安装期/构建期用，保持 fail-closed 契约。
#   默认(0)   失败只告警并返回 0；开机自愈路径用——本脚本挂掉不该连坐拖垮 dexe-auto
#             下的导航/传感器/command_board 等全部节点，真出问题时 ukf_node 自身的
#             fail-closed 日志仍会明确报出。
#
# 需要 root（setcap 要 CAP_SETFCAP）。systemd drop-in 用 `+` 前缀获得 root，
# 因此不需要明文 sudo 密码，也不需要给 dexforce 配 NOPASSWD setcap 规则
# （后者等价于把 root 交出去：sudo setcap cap_setuid=ep <任意二进制> 即可提权）。

INSTALL_DIR="${1:-/home/dexforce/w1/install}"
STRICT="${STRICT:-0}"
exit_code=0

note_failure() {
    echo "$1" >&2
    exit_code=1
}

for node_name in ukf_node ekf_node; do
    node_path="${INSTALL_DIR}/robot_localization/lib/robot_localization/${node_name}"

    if [ ! -f "${node_path}" ]; then
        note_failure "robot_localization node not found: ${node_path}"
        continue
    fi

    if ! setcap 'cap_sys_nice=+ep' "${node_path}"; then
        note_failure "setcap failed: ${node_path}"
        continue
    fi

    capabilities="$(getcap "${node_path}" 2>/dev/null)"
    case "${capabilities}" in
        *cap_sys_nice=ep*)
            # 每次启动都留一行可核对的证据，便于事后从 journal 判断权限链是否成立。
            echo "Configured realtime capability: ${capabilities}"
            ;;
        *)
            note_failure "CAP_SYS_NICE verification failed: ${node_path} (actual=${capabilities:-none})"
            ;;
    esac
done

if [ "${STRICT}" = "1" ]; then
    exit "${exit_code}"
fi

exit 0
