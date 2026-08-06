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
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/robot_localization/ukf_node"
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/robot_localization/ekf_node"
cp /bin/true "${INSTALL_DIR}/robot_localization/lib/librl_lib.so"

DEXE_MOBILE_SLAM_INSTALL_DIR="${INSTALL_DIR}" bash "${SOURCE_DIR}/install.sh"

PACKAGE_ROOT="${SOURCE_DIR}/debian/dexe-mobile-slam"
POSTINST="${PACKAGE_ROOT}/DEBIAN/postinst"
CONTROL="${PACKAGE_ROOT}/DEBIAN/control"

test -x "${POSTINST}"
cmp -s "${SOURCE_DIR}/postinst.sh" "${POSTINST}"
grep -Fq 'Depends: libcap2-bin' "${CONTROL}"

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

echo 'PASS: deb postinst configures and verifies robot_localization realtime capability'
