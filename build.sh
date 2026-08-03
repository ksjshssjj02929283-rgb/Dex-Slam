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

for command_name in sudo setcap getcap ldconfig ldd; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "❌ 缺少命令：${command_name}"
        [ "${command_name}" != "setcap" ] && [ "${command_name}" != "getcap" ] || \
            echo "   请先安装 libcap2-bin。"
        exit 1
    fi
done

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
sudo -v
sudo install -m 0644 "${LDCONF_TMP}" /etc/ld.so.conf.d/dexe_mobile_slam.conf
sudo ldconfig

resolved_rl_library="$(ldd "${UKF_NODE}" | awk '$1 == "librl_lib.so" {print $3; exit}')"
if [ -z "${resolved_rl_library}" ] || \
   [ "$(readlink -f "${resolved_rl_library}")" != "$(readlink -f "${RL_LIBRARY}")" ]; then
    echo "❌ ukf_node 未解析到本轮构建的 librl_lib.so"
    echo "   expected=$(readlink -f "${RL_LIBRARY}")"
    echo "   actual=${resolved_rl_library:-unresolved}"
    exit 1
fi

apply_realtime_capability() {
    local executable="$1"
    sudo setcap 'cap_sys_nice=+ep' "${executable}"
    local capabilities
    capabilities="$(getcap "${executable}")"
    if [[ "${capabilities}" != *"cap_sys_nice=ep"* ]]; then
        echo "❌ CAP_SYS_NICE 校验失败：${executable}"
        echo "   actual=${capabilities:-none}"
        exit 1
    fi
    echo "✅ ${capabilities}"
}

apply_realtime_capability "${UKF_NODE}"
apply_realtime_capability "${EKF_NODE}"
echo "✅ robot_localization 构建、动态库注册和实时权限配置完成。"
