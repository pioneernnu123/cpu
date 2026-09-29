#!/bin/bash

# CPU_SYSFS_ROOT may point to a temporary fixture directory in automated tests.
CPU_SYSFS_ROOT="${CPU_SYSFS_ROOT:-/sys/devices/system/cpu}"

offline_sensor_core() {
    local core_id="$1" kind="$2" online_file before after reason=""
    if [[ ! "$core_id" =~ ^[1-9][0-9]*$ ]]; then
        reason="invalid_or_boot_cpu"
    else
        online_file="$CPU_SYSFS_ROOT/cpu$core_id/online"
        if [[ ! -f "$online_file" ]]; then
            reason="online_interface_missing"
        elif ! IFS= read -r before < "$online_file"; then
            reason="online_read_failed"
        elif [[ "$before" != 1 ]]; then
            reason="cpu_not_online_before_injection"
        elif ! printf '0\n' > "$online_file"; then
            reason="offline_write_failed"
        elif ! IFS= read -r after < "$online_file" || [[ "$after" != 0 ]]; then
            reason="offline_readback_failed"
        fi
    fi
    if [[ -n "$reason" ]]; then
        printf '[!] CPU %s offline failed: %s\n' "$core_id" "$reason"
        printf 'INTERCEPT_RESULT kind=%s core=%s status=failure reason=%s\n' "$kind" "$core_id" "$reason"
        return 1
    fi
    printf '[+] CPU %s offline confirmed (online=0).\n' "$core_id"
    printf 'INTERCEPT_RESULT kind=%s core=%s status=success\n' "$kind" "$core_id"
}

# Allow the offline handler to be tested without starting a detector.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    return 0
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || exit 1
echo '[+] Hardware Fault Tolerance Bash Scheduler Started'
echo 'Detector daemon is launching...'

if [[ ! -x "$script_dir/detector" ]]; then
    echo Detector executable not found
    exit 1
fi

buffer=""
core_id=""
fault_type=""

exec {detector_output}< <(exec "$script_dir/detector")
detector_pid=$!
cleanup() {
    kill -TERM "$detector_pid" 2>/dev/null || true
    wait "$detector_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

while IFS= read -r line <&"$detector_output"; do
    printf '%s\n' "$line"

    clean_line=$(echo "$line" | sed -r 's/\x1B\[[0-9;]*[mK]//g')
    # Only collect a complete report, not periodic daemon heartbeat messages.
    if [[ "$clean_line" == *"╔"* ]]; then
        buffer=""
        core_id=""
        fault_type=""
        collecting=1
    fi
    if [[ "${collecting:-0}" != 1 ]]; then
        continue
    fi
    buffer="$buffer$clean_line"

    if [[ "$clean_line" == *"╝"* ]]; then
        if [[ "$buffer" =~ Core[[:space:]]+-[[:space:]]+Core[[:space:]]+([0-9]+) ]]; then
            core_id="${BASH_REMATCH[1]}"
        fi

        if [[ "$buffer" =~ Fault[[:space:]]+Type[[:space:]]+-[[:space:]]+(.*) ]]; then
            fault_type="${BASH_REMATCH[1]%%║*}"
            fault_type=$(echo "$fault_type" | xargs)
        fi

        if [[ -n "$core_id" && -n "$fault_type" ]]; then
            if [[ "$fault_type" == "CPU Temperature Above Limit" ]]; then
                offline_sensor_core "$core_id" temperature || true
            elif [[ "$fault_type" == "CPU Frequency Below Limit" ]]; then
                offline_sensor_core "$core_id" frequency || true
            elif [[ "$fault_type" == *"FPU"* || "$fault_type" == *"SIMD"* || "$fault_type" == *"MXCSR"* ]]; then
                echo
                echo [+] NON-FATAL ERROR INTERCEPTED - Core $core_id Fault - $fault_type
                echo [+] Initiating soft downgrade and power suppression

                cg_path="/sys/fs/cgroup/downgrade_$core_id"
                mkdir -p "$cg_path" 2>/dev/null
                echo "20000 100000" > "$cg_path/cpu.max" 2>/dev/null || echo 20000 > "/sys/fs/cgroup/cpu/downgrade_$core_id/cpu.cfs_quota_us" 2>/dev/null
                echo powersave > "/sys/devices/system/cpu/cpu$core_id/cpufreq/scaling_governor" 2>/dev/null

                echo "[+] Soft downgrade requested for CPU $core_id; verify quota and frequency settings before counting success"
            else
                echo
                echo [+] FATAL ERROR INTERCEPTED - Core $core_id Fault - $fault_type
                echo [+] Initiating physical cpuset quarantine

                if [[ -f /sys/fs/cgroup/cgroup.controllers ]]; then
                    cg_path="/sys/fs/cgroup/fatal_quarantine_$core_id"
                    mkdir -p "$cg_path" 2>/dev/null
                    echo "+cpuset" > /sys/fs/cgroup/cgroup.subtree_control 2>/dev/null
                    cat /sys/fs/cgroup/cpuset.mems.effective > "$cg_path/cpuset.mems" 2>/dev/null
                    echo "$core_id" > "$cg_path/cpuset.cpus" 2>/dev/null
                else
                    cg_path="/sys/fs/cgroup/cpuset/fatal_quarantine_$core_id"
                    mkdir -p "$cg_path" 2>/dev/null
                    cat /sys/fs/cgroup/cpuset/cpuset.mems > "$cg_path/cpuset.mems" 2>/dev/null
                    echo "$core_id" > "$cg_path/cpuset.cpus" 2>/dev/null
                    echo 1 > "$cg_path/cpuset.cpu_exclusive" 2>/dev/null
                fi

                echo "[+] Quarantine cpuset configuration requested for CPU $core_id; verify task placement before counting success"
            fi
        else
            echo
            echo [DEBUG] Regex Match Failed Buffer Content Follows
            echo "$buffer"
        fi

        buffer=""
        collecting=0
        core_id=""
        fault_type=""
    fi
done
exec {detector_output}<&-
wait "$detector_pid"
detector_status=$?
trap - EXIT
exit "$detector_status"
