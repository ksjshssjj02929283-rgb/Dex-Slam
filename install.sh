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

# --- 自动生成 postinst:整棵 /home/dexforce/w1 chown 回 dexforce ---
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"
mkdir -p "$DEBIAN_DIR"
cat > "$DEBIAN_DIR/postinst" <<'EOF'
#!/bin/sh
set -e
chown -R dexforce:dexforce /home/dexforce/w1
EOF
chmod 0755 "$DEBIAN_DIR/postinst"

# --- 自动生成 control:固定版本号 ---
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: 0.3.1
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PACKAGE_NAME
EOF
