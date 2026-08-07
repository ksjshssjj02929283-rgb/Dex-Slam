#!/bin/bash
set -e

SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "${SOURCE_DIR}/.." && pwd)"
INSTALL_DIR="${WORKSPACE_DIR}/install"

source /opt/ros/humble/setup.bash

# 输出环境中已有的 ROS 包名字，检查是否有 nav_interfaces 包。
if ros2 pkg list | grep -q "^nav_interfaces$"; then
    echo "✅ 找到 nav_interfaces 包，复用当前环境。"
else
    echo "⚠️ 未找到 nav_interfaces 包，将从 mobile SLAM 工作区构建。"
    if [ -f "${SOURCE_DIR}/.gitmodules" ]; then
        echo "🔄 检测到 .gitmodules，正在同步子模块..."
        git -C "${SOURCE_DIR}" submodule sync --recursive
        git -C "${SOURCE_DIR}" submodule update --init --recursive
        echo "✅ 子模块已更新完成。"
    else
        echo "ℹ️ 未检测到 .gitmodules，跳过子模块更新。"
    fi
fi

cd "${WORKSPACE_DIR}"
colcon build --base-paths "${SOURCE_DIR}" "$@"

UKF_NODE="${INSTALL_DIR}/robot_localization/lib/robot_localization/ukf_node"
EKF_NODE="${INSTALL_DIR}/robot_localization/lib/robot_localization/ekf_node"
RL_LIBRARY="${INSTALL_DIR}/robot_localization/lib/librl_lib.so"

REQUIRED_COMMANDS=(setcap getcap ldconfig ldd install)
if [ "${EUID}" -ne 0 ]; then
    REQUIRED_COMMANDS+=(sudo)
fi

for command_name in "${REQUIRED_COMMANDS[@]}"; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "❌ 缺少命令：${command_name}"
        [ "${command_name}" != "setcap" ] && [ "${command_name}" != "getcap" ] || \
            echo "   请先安装 libcap2-bin。"
        exit 1
    fi
done

run_privileged() {
    if [ "${EUID}" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

for required_file in "${UKF_NODE}" "${EKF_NODE}" "${RL_LIBRARY}"; do
    if [ ! -f "${required_file}" ]; then
        echo "❌ robot_localization 构建产物不存在：${required_file}"
        exit 1
    fi
done

# file capability 会触发 glibc secure-execution，LD_LIBRARY_PATH 将被忽略。
# 参考 robot server 的构建脚本，把本工作区 install 下的库目录注册给 ldconfig，
# 并在 setcap 前验证 ukf_node 确实解析到本轮构建的 librl_lib.so。
LDCONF_TMP="$(mktemp)"
trap 'rm -f "${LDCONF_TMP}"' EXIT
{
    for ros_lib_dir in \
        /opt/ros/humble/lib \
        /opt/ros/humble/lib/aarch64-linux-gnu \
        /opt/ros/humble/lib/x86_64-linux-gnu; do
        [ -d "${ros_lib_dir}" ] && echo "${ros_lib_dir}"
    done
    for package_lib_dir in "${INSTALL_DIR}"/*/lib; do
        [ -d "${package_lib_dir}" ] && echo "${package_lib_dir}"
    done
} > "${LDCONF_TMP}"

echo "🔐 构建完成，配置 robot_localization 实时调度权限..."
if [ "${EUID}" -ne 0 ]; then
    sudo -v
fi
run_privileged install -m 0644 "${LDCONF_TMP}" /etc/ld.so.conf.d/dexe_mobile_slam.conf
run_privileged ldconfig

resolved_rl_library="$(ldd "${UKF_NODE}" | awk '$1 == "librl_lib.so" {print $3; exit}')"
if [ -z "${resolved_rl_library}" ] || \
   [ "$(readlink -f "${resolved_rl_library}")" != "$(readlink -f "${RL_LIBRARY}")" ]; then
    echo "❌ ukf_node 未解析到本轮构建的 librl_lib.so"
    echo "   expected=$(readlink -f "${RL_LIBRARY}")"
    echo "   actual=${resolved_rl_library:-unresolved}"
    exit 1
fi

# setcap 逻辑与 deb postinst / dexe-auto drop-in 共用同一份脚本，避免多处重复。
ENSURE_CAPS_SCRIPT="${SOURCE_DIR}/ensure_realtime_caps.sh"
if [ ! -x "${ENSURE_CAPS_SCRIPT}" ]; then
    echo "❌ 缺少 capability 脚本或未置可执行位：${ENSURE_CAPS_SCRIPT}"
    exit 1
fi
# STRICT=1：构建期保持 fail-closed。sudo 默认不透传环境变量，所以用 env 传。
run_privileged env STRICT=1 "${ENSURE_CAPS_SCRIPT}" "${INSTALL_DIR}"

# 共享部署树上还有 20+ 个兄弟包的 deb postinst 会 `chown -R /home/dexforce/w1`，
# 而 chown(2) 对非目录无条件清除 security.capability。源码工作区构建同样会被后续任何一次
# 兄弟包安装抹掉，因此在有 dexe-auto.service 的机器上一并下发自愈 drop-in（调用同一个
# 脚本），让每次服务启动前重新授权。开发机没有该 unit，自动跳过。
if systemctl cat dexe-auto.service >/dev/null 2>&1; then
    SERVICE_DROPIN=/etc/systemd/system/dexe-auto.service.d/10-dexe-slam-realtime.conf
    DROPIN_TMP="$(mktemp)"
    cat > "${DROPIN_TMP}" <<EOF
[Service]
ExecStartPre=-+${ENSURE_CAPS_SCRIPT} ${INSTALL_DIR}
EOF
    run_privileged mkdir -p "$(dirname "${SERVICE_DROPIN}")"
    run_privileged install -m 0644 "${DROPIN_TMP}" "${SERVICE_DROPIN}"
    rm -f "${DROPIN_TMP}"
    run_privileged systemctl daemon-reload || true
    echo "✅ 已下发实时权限自愈 drop-in：${SERVICE_DROPIN}"
else
    echo "ℹ️ 未检测到 dexe-auto.service，跳过实时权限自愈 drop-in（开发机正常现象）。"
fi

echo "✅ robot_localization 构建、动态库注册和实时权限配置完成。"
