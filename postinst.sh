#!/bin/sh
# dexe-mobile-slam deb 安装后脚本（CI 与本地打包的单一来源）。
# 必须在 deb 已完成文件落盘后，对最终 UKF/EKF ELF 设置 capability；任何后续
# chown、strip、重新链接或覆盖都会清除 file capability。
set -e

ROOT="${DPKG_ROOT:-}"
INSTALL_PATH="${ROOT}/home/dexforce/w1/install"
SCRIPTS_PATH="${ROOT}/home/dexforce/w1/dexe-mobile-slam"
UKF_NODE="${INSTALL_PATH}/robot_localization/lib/robot_localization/ukf_node"
EKF_NODE="${INSTALL_PATH}/robot_localization/lib/robot_localization/ekf_node"
RL_LIBRARY="${INSTALL_PATH}/robot_localization/lib/librl_lib.so"

# setcap 逻辑的单一来源；本包随 deb 安装到 SCRIPTS_PATH。
ENSURE_CAPS_SCRIPT="${SCRIPTS_PATH}/ensure_realtime_caps.sh"
# 目标机运行期的真实路径（不带 DPKG_ROOT）；systemd drop-in 的内容必须用它。
RUNTIME_ENSURE_CAPS_SCRIPT="/home/dexforce/w1/dexe-mobile-slam/ensure_realtime_caps.sh"
SERVICE_DROPIN="${ROOT}/etc/systemd/system/dexe-auto.service.d/10-dexe-slam-realtime.conf"

# 只调整本包脚本目录。chown 会清除可执行文件已有的 file capability，
# 因此不能再对共享的 /home/dexforce/w1 整树执行 chown。
if [ -d "${SCRIPTS_PATH}" ]; then
    chown -R dexforce:dexforce "${SCRIPTS_PATH}"
fi

for command_name in setcap getcap ldconfig ldd; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Error: required command not found: ${command_name}" >&2
        [ "${command_name}" != "setcap" ] && [ "${command_name}" != "getcap" ] || \
            echo "  Install libcap2-bin, then reinstall dexe-mobile-slam." >&2
        exit 1
    fi
done

for required_file in "${UKF_NODE}" "${EKF_NODE}" "${RL_LIBRARY}"; do
    if [ ! -f "${required_file}" ]; then
        echo "Error: robot_localization runtime file not found: ${required_file}" >&2
        exit 1
    fi
done

# file capability 会使 glibc 进入 secure-execution，LD_LIBRARY_PATH 不再可靠。
# 先注册 deb 最终安装目录中的动态库，再校验 ukf_node 的实际解析结果。
LDCONF_FILE="${ROOT}/etc/ld.so.conf.d/dexe_mobile_slam.conf"
mkdir -p "$(dirname "${LDCONF_FILE}")"
{
    for ros_lib_dir in \
        /opt/ros/humble/lib \
        /opt/ros/humble/lib/aarch64-linux-gnu \
        /opt/ros/humble/lib/x86_64-linux-gnu; do
        if [ -d "${ROOT}${ros_lib_dir}" ]; then
            printf '%s\n' "${ros_lib_dir}"
        fi
    done

    find "${INSTALL_PATH}" -mindepth 2 -maxdepth 2 -type d -name lib | while IFS= read -r package_lib_dir; do
        if [ -n "${ROOT}" ]; then
            printf '%s\n' "${package_lib_dir#"${ROOT}"}"
        else
            printf '%s\n' "${package_lib_dir}"
        fi
    done
} | sort -u > "${LDCONF_FILE}"

if [ -n "${ROOT}" ]; then
    ldconfig -r "${ROOT}"
else
    ldconfig
fi

resolved_rl_library="$(ldd "${UKF_NODE}" | awk '$1 == "librl_lib.so" {print $3; exit}')"
if [ -z "${resolved_rl_library}" ] || \
   [ "$(readlink -f "${resolved_rl_library}")" != "$(readlink -f "${RL_LIBRARY}")" ]; then
    echo "Error: ukf_node did not resolve the packaged librl_lib.so" >&2
    echo "  expected=$(readlink -f "${RL_LIBRARY}")" >&2
    echo "  actual=${resolved_rl_library:-unresolved}" >&2
    exit 1
fi

if [ ! -x "${ENSURE_CAPS_SCRIPT}" ]; then
    echo "Error: capability helper not found or not executable: ${ENSURE_CAPS_SCRIPT}" >&2
    exit 1
fi

# 必须是 postinst 的最后一个文件元数据操作；后续 chown/strip/覆盖都会清除 capability。
# STRICT=1：安装期保持 fail-closed，授权或校验失败即让 dpkg 配置阶段非零退出。
STRICT=1 "${ENSURE_CAPS_SCRIPT}" "${INSTALL_PATH}"

# 共享部署树 /home/dexforce/w1 上还有 20+ 个兄弟包的 postinst 会执行
# `chown -R dexforce:dexforce /home/dexforce/w1`，而 chown(2) 对非目录无条件清除
# security.capability（chown 到相同 owner 也清）。本包安装完成后只要再装/升级任何一个
# 兄弟包，上面刚写好的 capability 就会被抹掉。
# 因此额外下发 systemd drop-in，让 dexe-auto 每次启动前重新跑同一个脚本，与安装顺序解耦。
#   `+` 前缀：以 root 执行，绕过 unit 的 User=dexforce（因此不需要明文 sudo 密码，
#             也不需要给 dexforce 配 NOPASSWD setcap 规则——后者等价于交出 root）。
#   `-` 前缀：失败不阻断整个 dexe-auto（否则本包被卸载或路径变更会连带拖垮导航/传感器
#             等全部节点）；真出问题时 ukf_node 自身的 fail-closed 日志仍会明确报出。
# 脚本不传 STRICT，走宽松模式；每轮的 getcap 结果仍会打进 journal 供事后核对。
mkdir -p "$(dirname "${SERVICE_DROPIN}")"
cat > "${SERVICE_DROPIN}" <<EOF
[Service]
ExecStartPre=-+${RUNTIME_ENSURE_CAPS_SCRIPT}
EOF
chmod 0644 "${SERVICE_DROPIN}"
echo "Installed realtime capability drop-in: ${SERVICE_DROPIN}"

if [ -z "${ROOT}" ] && command -v systemctl >/dev/null 2>&1; then
    systemctl daemon-reload || true
    # drop-in 挂在别的包提供的 unit 上。unit 不存在时 systemd 会静默忽略整个 drop-in
    # 目录，权限自愈也就不会发生——这里显式告警，避免 unit 改名/重构后无声失效。
    # 只告警不失败：本包可能先于 dexe_mobile_application 安装，届时 unit 尚不存在。
    if ! systemctl cat dexe-auto.service >/dev/null 2>&1; then
        echo "Warning: dexe-auto.service not found; the realtime drop-in stays inert until that unit exists." >&2
    fi
fi

exit 0
