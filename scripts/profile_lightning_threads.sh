#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  profile_lightning_threads.sh attach [seconds]
  profile_lightning_threads.sh launch [seconds] -- <command> [args...]

Modes:
  attach  Verify that run_loc_online is running, then record system-wide CPU
          samples and thread scheduling; sudo is required.
  launch  Launch a command under Nsight Systems. This additionally records the
          NVTX ranges when Lightning was built with ENABLE_NVTX_PROFILING=ON.
          The command is terminated when the requested profiling duration ends.

Reports are written to: $HOME/lightning_profiles
EOF
}

mode="${1:-attach}"
if [[ "$#" -gt 0 ]]; then
    shift
fi

duration=30
if [[ "${1:-}" =~ ^[1-9][0-9]*$ ]]; then
    duration="$1"
    shift
fi

if ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
    echo "Duration must be a positive integer, got: $duration" >&2
    exit 2
fi

case "$mode" in
    -h|--help|help)
        usage
        exit 0
        ;;
esac

if ! command -v nsys >/dev/null 2>&1; then
    echo "nsys was not found. Install NVIDIA Nsight Systems first." >&2
    exit 1
fi

output_dir="${LIGHTNING_PROFILE_DIR:-${HOME}/lightning_profiles}"
mkdir -p "$output_dir"
timestamp="$(date +%Y%m%d-%H%M%S)"
session="lightning_${timestamp//-/_}"
output_base="${output_dir}/${session}"

case "$mode" in
    attach)
        if [[ "$#" -ne 0 ]]; then
            usage >&2
            exit 2
        fi

        target_pid="$(pgrep -n -x run_loc_online || true)"
        if [[ -z "$target_pid" ]]; then
            echo "run_loc_online is not running." >&2
            exit 1
        fi

        echo "Profiling existing run_loc_online PID ${target_pid} for ${duration}s"
        echo "Report base: ${output_base}"
        echo "sudo is required for system-wide CPU scheduling data."

        stopped=0
        stop_session() {
            if [[ "$stopped" -eq 0 ]]; then
                sudo nsys stop --session="$session" >/dev/null || true
                stopped=1
            fi
        }
        trap stop_session EXIT INT TERM

        sudo nsys start \
            --session-new="$session" \
            --sample=system-wide \
            --cpuctxsw=system-wide \
            --backtrace=dwarf \
            --force-overwrite=true \
            --output="$output_base"

        sleep "$duration"
        stop_session
        trap - EXIT INT TERM
        find "$output_dir" -maxdepth 1 -type f -name "${session}*" \
            -exec sudo chown "$(id -u):$(id -g)" {} +
        ;;

    launch)
        if [[ "${1:-}" == "--" ]]; then
            shift
        fi
        if [[ "$#" -eq 0 ]]; then
            usage >&2
            exit 2
        fi

        echo "Launching under Nsight Systems for ${duration}s: $*"
        echo "The launched process group will receive SIGTERM when profiling ends."
        # Keep Nsight and its target in a separate process group. Otherwise
        # --kill=sigterm may also terminate this wrapper in non-interactive SSH.
        set +e
        setsid nsys profile \
            --duration="$duration" \
            --kill=sigterm \
            --sample=process-tree \
            --cpuctxsw=process-tree \
            --backtrace=dwarf \
            --trace=osrt,nvtx,cuda \
            --force-overwrite=true \
            --output="$output_base" \
            "$@"
        profile_status=$?
        set -e
        if [[ "$profile_status" -ne 0 && "$profile_status" -ne 143 ]]; then
            exit "$profile_status"
        fi
        ;;

    *)
        usage >&2
        exit 2
        ;;
esac

report="$(find "$output_dir" -maxdepth 1 -type f -name "${session}*.nsys-rep" -print -quit)"
if [[ -n "$report" ]]; then
    echo "Profile complete: $report"
    echo "Open it with: nsys-ui '$report'"
else
    echo "Profiling finished, but no .nsys-rep file was found under $output_dir" >&2
    exit 1
fi
