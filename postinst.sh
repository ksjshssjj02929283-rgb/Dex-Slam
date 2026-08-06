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

apply_realtime_capability() {
    executable="$1"
    setcap 'cap_sys_nice=+ep' "${executable}"
    capabilities="$(getcap "${executable}")"
    case "${capabilities}" in
        *cap_sys_nice=ep*)
            echo "Configured realtime capability: ${capabilities}"
            ;;
        *)
            echo "Error: CAP_SYS_NICE verification failed: ${executable}" >&2
            echo "  actual=${capabilities:-none}" >&2
            exit 1
            ;;
    esac
}

# 必须是 postinst 的最后一个文件元数据操作；后续 chown/strip/覆盖都会清除 capability。
apply_realtime_capability "${UKF_NODE}"
apply_realtime_capability "${EKF_NODE}"

exit 0
