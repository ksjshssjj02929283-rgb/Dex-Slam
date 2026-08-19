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

# --- 剔除 dexe_interfaces 接口包: 机上接口由 dexe-interfaces deb 铺设, 本包不重复发布 ---
# 包名以 package.xml 的 <name> 为准, 目录名与包名可能不一致(dexe_interfaces_meta -> dexe_interfaces)。
# colcon 的 install/setup.bash 动态枚举包目录, 删子目录不影响 source。
if [ -d "$CURDIR/dexe_interfaces" ]; then
    for xml in "$CURDIR"/dexe_interfaces/*/package.xml; do
        [ -f "$xml" ] || continue
        pkg=$(sed -n 's|.*<name>\([^<]*\)</name>.*|\1|p' "$xml" | head -n1)
        [ -n "$pkg" ] || continue
        echo "Excluding interface package from deb: $pkg"
        rm -rf "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install/$pkg"
    done
fi

# setcap 逻辑的单一来源：postinst 首次授权、dexe-auto drop-in 每次启动自愈、
# 以及开发者手工排障（如 debug_localization.sh 之前）都调用这一份。
if [ ! -f "$CURDIR/ensure_realtime_caps.sh" ]; then
    echo "Error: ensure_realtime_caps.sh not found: $CURDIR/ensure_realtime_caps.sh"
    exit 1
fi
cp "$CURDIR/ensure_realtime_caps.sh" "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME/"
chmod 0755 "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME/ensure_realtime_caps.sh"

# --- DEBIAN/postinst + postrm：CI 与本地打包复用仓库根单一来源 ---
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"
mkdir -p "$DEBIAN_DIR"
for maintainer_script in postinst postrm; do
    if [ ! -f "$CURDIR/${maintainer_script}.sh" ]; then
        echo "Error: ${maintainer_script}.sh not found: $CURDIR/${maintainer_script}.sh"
        exit 1
    fi
    cp "$CURDIR/${maintainer_script}.sh" "$DEBIAN_DIR/${maintainer_script}"
    chmod 0755 "$DEBIAN_DIR/${maintainer_script}"
done

# --- 自动生成 control:固定版本号 ---
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: 0.3.1
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Depends: libcap2-bin
Description: $PACKAGE_NAME
EOF
