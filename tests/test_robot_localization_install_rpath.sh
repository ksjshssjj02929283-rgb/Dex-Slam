#!/bin/bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PACKAGE_PREFIX="${1:-${REPO_DIR}/install/robot_localization}"
LIB_DIR="${PACKAGE_PREFIX}/lib"

read_runpath() {
  readelf -d "$1" | sed -n \
    's/.*(RUNPATH).*Library runpath: \[\(.*\)\]/\1/p'
}

assert_runpath() {
  local elf=$1
  local expected=$2
  local actual=""

  actual="$(read_runpath "${elf}")"
  if [ "${actual}" != "${expected}" ]; then
    printf 'FAIL: %s RUNPATH expected=%s actual=%s\n' \
      "${elf}" "${expected}" "${actual:-missing}" >&2
    return 1
  fi
}

assert_resolves_packaged_rl() {
  local node=$1
  local expected=""
  local actual=""

  expected="$(readlink -f "${LIB_DIR}/librl_lib.so")"
  actual="$(
    env -u LD_LIBRARY_PATH ldd "${node}" |
      awk '$1 == "librl_lib.so" && $2 == "=>" {print $3; exit}'
  )"
  if [ -z "${actual}" ] || [ "${actual}" = "not" ] || \
     [ "$(readlink -f "${actual}")" != "${expected}" ]; then
    printf 'FAIL: %s librl_lib.so expected=%s actual=%s\n' \
      "${node}" "${expected}" "${actual:-unresolved}" >&2
    return 1
  fi
}

assert_runpath "${LIB_DIR}/librl_lib.so" '$ORIGIN'
for node_name in ukf_node ekf_node; do
  node="${LIB_DIR}/robot_localization/${node_name}"
  assert_runpath "${node}" '$ORIGIN/..'
  assert_resolves_packaged_rl "${node}"
done

echo 'PASS: installed UKF/EKF resolve packaged librl_lib.so through relative RUNPATH'
