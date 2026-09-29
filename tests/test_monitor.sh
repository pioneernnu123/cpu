#!/bin/bash
set -euo pipefail
base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fixture="$(mktemp -d "$base/tests/.tmp-monitor.XXXXXX")"
fixture="$(cd -- "$fixture" && pwd)"
cleanup_fixture() {
    case "$fixture" in
        "$base/tests/.tmp-monitor."*) rm -rf -- "$fixture" ;;
        *) printf 'Refusing cleanup outside test directory: %s\n' "$fixture" >&2 ;;
    esac
}
trap cleanup_fixture EXIT
export CPU_SYSFS_ROOT="$fixture/sys/cpu"
mkdir -p "$CPU_SYSFS_ROOT/cpu0" "$CPU_SYSFS_ROOT/cpu1" "$CPU_SYSFS_ROOT/cpu2"
source "$base/monitor.sh"
printf '1\n' > "$CPU_SYSFS_ROOT/cpu0/online"

for kind in temperature frequency; do
    for run in 1 2 3 4 5; do
        printf '1\n' > "$CPU_SYSFS_ROOT/cpu1/online"
        output="$(offline_sensor_core 1 "$kind")"
        [[ "$output" == *"INTERCEPT_RESULT kind=$kind core=1 status=success"* ]]
        [[ "$(cat "$CPU_SYSFS_ROOT/cpu1/online")" == 0 ]]
    done
done
if output="$(offline_sensor_core 1 temperature)"; then exit 1; fi
[[ "$output" == *'status=failure reason=cpu_not_online_before_injection'* ]]
if output="$(offline_sensor_core 0 temperature)"; then exit 1; fi
[[ "$output" == *'status=failure reason=invalid_or_boot_cpu'* ]]
[[ "$(cat "$CPU_SYSFS_ROOT/cpu0/online")" == 1 ]]
if output="$(offline_sensor_core '../cpu0' frequency)"; then exit 1; fi
[[ "$output" == *'status=failure reason=invalid_or_boot_cpu'* ]]
if output="$(offline_sensor_core 99 frequency)"; then exit 1; fi
[[ "$output" == *'status=failure reason=online_interface_missing'* ]]

# Emulate rejected sysfs writes and writes that do not change online state.
printf '1\n' > "$CPU_SYSFS_ROOT/cpu1/online"
if output="$(
    printf() { if [[ "$1" == '0\n' ]]; then return 1; else builtin printf "$@"; fi; }
    offline_sensor_core 1 temperature
)"; then exit 1; fi
[[ "$output" == *'status=failure reason=offline_write_failed'* ]]
printf '1\n' > "$CPU_SYSFS_ROOT/cpu1/online"
if output="$(
    printf() { if [[ "$1" == '0\n' ]]; then builtin printf '1\n'; else builtin printf "$@"; fi; }
    offline_sensor_core 1 frequency
)"; then exit 1; fi
[[ "$output" == *'status=failure reason=offline_readback_failed'* ]]

# Run the real scheduler with a fake report producer and temporary CPU files.
cp "$base/monitor.sh" "$fixture/monitor.sh"
cat > "$fixture/detector" <<'DETECTOR'
#!/bin/bash
printf 'Detection running...\r\n'
printf '\033[1;31m╔════════════════════╗\033[0m\n'
printf '║ Core       - Core 1 ║\n'
printf '║ Fault Type - CPU Temperature Above Limit ║\n'
printf '╚════════════════════╝\n'
printf 'Detection running...\r\n'
printf '╔════════════════════╗\n'
printf '║ Core       - Core 2 ║\n'
printf '║ Fault Type - CPU Frequency Below Limit ║\n'
printf '╚════════════════════╝\n'
DETECTOR
chmod +x "$fixture/detector"
printf '1\n' > "$CPU_SYSFS_ROOT/cpu1/online"
printf '1\n' > "$CPU_SYSFS_ROOT/cpu2/online"
output="$(bash "$fixture/monitor.sh")"
[[ "$(printf '%s\n' "$output" | grep -c '^INTERCEPT_RESULT .*status=success')" == 2 ]]
[[ "$output" == *'kind=temperature core=1 status=success'* ]]
[[ "$output" == *'kind=frequency core=2 status=success'* ]]
[[ "$(cat "$CPU_SYSFS_ROOT/cpu1/online")" == 0 ]]
[[ "$(cat "$CPU_SYSFS_ROOT/cpu2/online")" == 0 ]]
printf '%s\n' 'PASS: 5 temperature + 5 frequency dispositions, negative paths and complete report-to-monitor flow'
