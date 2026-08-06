#!/bin/bash
set -e

CURDIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "${CURDIR}/.." && pwd)"
INSTALL_DIR="${DEXE_MOBILE_SLAM_INSTALL_DIR:-${WORKSPACE_DIR}/install}"
PROJECT_DIR=$(basename "${CURDIR}")
PACKAGE_NAME=${PROJECT_DIR//_/-}
echo "Running file installation for $PACKAGE_NAME..."
echo "Using colcon install directory: ${INSTALL_DIR}"


mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME"
mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install"

if [ ! -d "${INSTALL_DIR}" ]; then
    echo "Error: install directory not found: ${INSTALL_DIR}"
    echo "Set DEXE_MOBILE_SLAM_INSTALL_DIR when packaging a non-default workspace."
    exit 1
fi

rm -rf "${INSTALL_DIR}/cartographer*"

find "${INSTALL_DIR}" -maxdepth 1 -type f -exec cp {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME/" \;
find "${INSTALL_DIR}" -maxdepth 1 -mindepth 1 -type d -exec cp -r {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install/" \;

# --- DEBIAN/postinst：CI 与本地打包复用仓库根单一来源 ---
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"
mkdir -p "$DEBIAN_DIR"
if [ ! -f "$CURDIR/postinst.sh" ]; then
    echo "Error: postinst.sh not found: $CURDIR/postinst.sh"
    exit 1
fi
cp "$CURDIR/postinst.sh" "$DEBIAN_DIR/postinst"
chmod 0755 "$DEBIAN_DIR/postinst"

# --- 自动生成 control:固定版本号 ---
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: 0.3.1
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Depends: libcap2-bin
Description: $PACKAGE_NAME
EOF
