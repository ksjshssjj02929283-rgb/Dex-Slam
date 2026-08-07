#!/bin/sh
# dexe-mobile-slam deb 卸载后脚本（CI 与本地打包的单一来源）。
# 唯一职责：移除 postinst 下发的 dexe-auto systemd drop-in。
# 不清理时不仅残留配置，drop-in 里的 ExecStartPre 还会指向已被删除的 ELF 路径。
set -e

ROOT="${DPKG_ROOT:-}"
SERVICE_DROPIN="${ROOT}/etc/systemd/system/dexe-auto.service.d/10-dexe-slam-realtime.conf"

case "$1" in
    remove|purge)
        if [ -f "${SERVICE_DROPIN}" ]; then
            rm -f "${SERVICE_DROPIN}"
            # drop-in 目录可能还有别的包的配置，非空则保留。
            rmdir "$(dirname "${SERVICE_DROPIN}")" 2>/dev/null || true
            echo "Removed realtime capability drop-in: ${SERVICE_DROPIN}"
        fi

        if [ -z "${ROOT}" ] && command -v systemctl >/dev/null 2>&1; then
            systemctl daemon-reload || true
        fi
        ;;
esac

exit 0
