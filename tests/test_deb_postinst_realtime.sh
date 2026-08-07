#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf "${TEST_ROOT:?}"
}
trap cleanup EXIT

SOURCE_DIR="${TEST_ROOT}/dexe_mobile_slam"
INSTALL_DIR="${TEST_ROOT}/install"
FAKE_BIN_DIR="${TEST_ROOT}/fake-bin"
CALL_LOG="${TEST_ROOT}/calls.log"

mkdir -p \
  "${SOURCE_DIR}" \
  "${INSTALL_DIR}/robot_localization/lib/robot_localization" \
  "${INSTALL_DIR}/robot_localization/lib" \
  "${FAKE_BIN_DIR}"

cp "${REPO_DIR}/install.sh" "${SOURCE_DIR}/install.sh"
cp "${REPO_DIR}/postinst.sh" "${SOURCE_DIR}/postinst.sh"
cp "${REPO_DIR}/postrm.sh" "${SOURCE_DIR}/postrm.sh"
cp "${REPO_DIR}/ensure_realtime_caps.sh" "${SOURCE_DIR}/ensure_realtime_caps.sh"
chmod 0755 "${SOURCE_DIR}/ensure_realtime_caps.sh"
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/robot_localization/ukf_node"
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/robot_localization/ekf_node"
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/librl_lib.so"

DEXE_MOBILE_SLAM_INSTALL_DIR="${INSTALL_DIR}" bash "${SOURCE_DIR}/install.sh"

PACKAGE_ROOT="${SOURCE_DIR}/debian/dexe-mobile-slam"
POSTINST="${PACKAGE_ROOT}/DEBIAN/postinst"
POSTRM="${PACKAGE_ROOT}/DEBIAN/postrm"
CONTROL="${PACKAGE_ROOT}/DEBIAN/control"

STAGED_HELPER="${PACKAGE_ROOT}/home/dexforce/w1/dexe-mobile-slam/ensure_realtime_caps.sh"

test -x "${POSTINST}"
test -x "${POSTRM}"
cmp -s "${SOURCE_DIR}/postinst.sh" "${POSTINST}"
cmp -s "${SOURCE_DIR}/postrm.sh" "${POSTRM}"
grep -Fq 'Depends: libcap2-bin' "${CONTROL}"

# setcap 逻辑必须只有一份：随包安装的 helper 与仓库根逐字一致，且可执行。
test -x "${STAGED_HELPER}"
cmp -s "${SOURCE_DIR}/ensure_realtime_caps.sh" "${STAGED_HELPER}"

cat > "${FAKE_BIN_DIR}/chown" <<'EOF'
#!/bin/sh
printf 'chown %s\n' "$*" >> "${CALL_LOG}"
EOF

cat > "${FAKE_BIN_DIR}/ldconfig" <<'EOF'
#!/bin/sh
printf 'ldconfig %s\n' "$*" >> "${CALL_LOG}"
EOF

cat > "${FAKE_BIN_DIR}/ldd" <<'EOF'
#!/bin/sh
printf 'ldd %s\n' "$*" >> "${CALL_LOG}"
printf 'librl_lib.so => %s/home/dexforce/w1/install/robot_localization/lib/librl_lib.so (0x0000)\n' "${DPKG_ROOT}"
EOF

cat > "${FAKE_BIN_DIR}/setcap" <<'EOF'
#!/bin/sh
printf 'setcap %s\n' "$*" >> "${CALL_LOG}"
EOF

cat > "${FAKE_BIN_DIR}/getcap" <<'EOF'
#!/bin/sh
printf 'getcap %s\n' "$*" >> "${CALL_LOG}"
printf '%s cap_sys_nice=ep\n' "$1"
EOF

chmod 0755 "${FAKE_BIN_DIR}"/*
: > "${CALL_LOG}"

PATH="${FAKE_BIN_DIR}:${PATH}" \
CALL_LOG="${CALL_LOG}" \
DPKG_ROOT="${PACKAGE_ROOT}" \
  "${POSTINST}" configure

UKF_NODE="${PACKAGE_ROOT}/home/dexforce/w1/install/robot_localization/lib/robot_localization/ukf_node"
EKF_NODE="${PACKAGE_ROOT}/home/dexforce/w1/install/robot_localization/lib/robot_localization/ekf_node"
LDCONF_FILE="${PACKAGE_ROOT}/etc/ld.so.conf.d/dexe_mobile_slam.conf"

grep -Fq "setcap cap_sys_nice=+ep ${UKF_NODE}" "${CALL_LOG}"
grep -Fq "setcap cap_sys_nice=+ep ${EKF_NODE}" "${CALL_LOG}"
grep -Fq "getcap ${UKF_NODE}" "${CALL_LOG}"
grep -Fq "getcap ${EKF_NODE}" "${CALL_LOG}"
grep -Fq 'ldconfig -r' "${CALL_LOG}"
grep -Fq '/home/dexforce/w1/install/robot_localization/lib' "${LDCONF_FILE}"

if grep -Fxq "chown -R dexforce:dexforce ${PACKAGE_ROOT}/home/dexforce/w1" "${CALL_LOG}"; then
  echo 'postinst must not chown the shared /home/dexforce/w1 tree' >&2
  exit 1
fi

chown_line="$(grep -n '^chown ' "${CALL_LOG}" | head -n 1 | cut -d: -f1)"
setcap_line="$(grep -n '^setcap ' "${CALL_LOG}" | head -n 1 | cut -d: -f1)"
test -n "${chown_line}"
test -n "${setcap_line}"
test "${chown_line}" -lt "${setcap_line}"

# --- 兄弟包 chown 共享树后仍要自愈：dexe-auto systemd drop-in ---
DROPIN="${PACKAGE_ROOT}/etc/systemd/system/dexe-auto.service.d/10-dexe-slam-realtime.conf"
RUNTIME_HELPER='/home/dexforce/w1/dexe-mobile-slam/ensure_realtime_caps.sh'

test -f "${DROPIN}"
grep -Fxq '[Service]' "${DROPIN}"

# drop-in 内容必须用目标机运行期真实路径，不能带 DPKG_ROOT 前缀。
if grep -Fq "${PACKAGE_ROOT}" "${DROPIN}"; then
  echo 'drop-in must reference runtime paths, not the staging DPKG_ROOT' >&2
  exit 1
fi

# `+` 以 root 执行绕过 User=dexforce；`-` 让失败不拖垮整个 dexe-auto。
if ! grep -Eq "^ExecStartPre=(-\+|\+-)${RUNTIME_HELPER}\$" "${DROPIN}"; then
  echo 'drop-in must invoke the shared capability helper with -+ prefixes' >&2
  cat "${DROPIN}" >&2
  exit 1
fi

# setcap 逻辑不得在 drop-in 里再内联一份（单一来源）。
if grep -q 'setcap' "${DROPIN}"; then
  echo 'drop-in must delegate to ensure_realtime_caps.sh, not inline setcap' >&2
  cat "${DROPIN}" >&2
  exit 1
fi

# --- helper 的 STRICT 语义：安装/构建期 fail-closed，开机自愈期宽松 ---
HELPER_PROBE_DIR="${TEST_ROOT}/helper-probe"
mkdir -p "${HELPER_PROBE_DIR}"
if PATH="${FAKE_BIN_DIR}:${PATH}" STRICT=1 "${STAGED_HELPER}" "${HELPER_PROBE_DIR}" >/dev/null 2>&1; then
  echo 'helper must fail closed under STRICT=1 when nodes are missing' >&2
  exit 1
fi
if ! PATH="${FAKE_BIN_DIR}:${PATH}" "${STAGED_HELPER}" "${HELPER_PROBE_DIR}" >/dev/null 2>&1; then
  echo 'helper must stay non-fatal without STRICT (boot self-heal path)' >&2
  exit 1
fi

# 卸载必须回收 drop-in，否则 ExecStartPre 会指向已删除的 helper。
DPKG_ROOT="${PACKAGE_ROOT}" "${POSTRM}" remove
if [ -f "${DROPIN}" ]; then
  echo 'postrm must remove the dexe-auto drop-in on package removal' >&2
  exit 1
fi

echo 'PASS: deb postinst configures and verifies robot_localization realtime capability'
echo 'PASS: dexe-auto drop-in re-applies capability on every start and is removed on purge'
