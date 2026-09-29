#ifndef CPU_FAULT_SHARED_H
#define CPU_FAULT_SHARED_H

#include <stdatomic.h>

/* Rebuild detector and injector together when this layout changes. */
#define SHM_NAME "/cpu_checker_shm_v2"
#define SHM_VERSION 2
#define MAX_CORES 1024

/* Synthetic values only: no thermal/hwmon/cpufreq interface is read. */
#define SIM_TEMP_NORMAL_MC 45000
#define SIM_TEMP_LIMIT_MC 80000
#define SIM_TEMP_FAULT_MC (SIM_TEMP_LIMIT_MC + 10000)
#define SIM_FREQ_NORMAL_MHZ 2000
#define SIM_FREQ_MIN_MHZ 1000
#define SIM_FREQ_FAULT_MHZ (SIM_FREQ_MIN_MHZ - 200)
#define SENSOR_POLL_US 100000

_Static_assert(SIM_TEMP_NORMAL_MC <= SIM_TEMP_LIMIT_MC, "Normal temperature exceeds limit");
_Static_assert(SIM_TEMP_FAULT_MC > SIM_TEMP_LIMIT_MC, "Temperature injection must exceed limit");
_Static_assert(SIM_FREQ_NORMAL_MHZ >= SIM_FREQ_MIN_MHZ, "Normal frequency is below limit");
_Static_assert(SIM_FREQ_FAULT_MHZ >= 0 && SIM_FREQ_FAULT_MHZ < SIM_FREQ_MIN_MHZ,
               "Frequency injection must be nonnegative and below limit");

typedef enum {
    HW_FAULT_NONE = 0,
    HW_FAULT_ALU_XOR_BITFLIP,
    HW_FAULT_ALU_SHIFTER_OFF1,
    HW_FAULT_ALU_ADDER_CARRY,
    HW_FAULT_ALU_MUL_PARTIAL,
    HW_FAULT_DIV_QUOTIENT_OFF1,
    HW_FAULT_DIV_HIGHREG_FLIP,
    HW_FAULT_FPU_MANTISSA_FLIP,
    HW_FAULT_FPU_EXPONENT_FLIP,
    HW_FAULT_FPU_ROUNDING_MODE,
    HW_FAULT_SIMD_SIGN_FLIP,
    HW_FAULT_FPU_NAN_PROPAGATE,
    HW_FAULT_CACHE_L1_BITFLIP,
    HW_FAULT_CTRL_LOOP_COUNTER,
    HW_FAULT_TEMP_HIGH,
    HW_FAULT_FREQ_LOW,
    HW_FAULT_MAX
} hw_fault_type_t;

typedef struct {
    hw_fault_type_t type;
    const char *short_name;
    const char *category;
    const char *mechanism;
} fault_desc_t;

static const fault_desc_t FAULT_TABLE[] = {
    { HW_FAULT_ALU_XOR_BITFLIP, "ALU Logic Gate SEU", "Class A Integer ALU", "Cosmic ray causes bit flip in ALU" },
    { HW_FAULT_ALU_SHIFTER_OFF1, "Barrel Shifter MUX Fault", "Class A Integer ALU", "Shift decoder MUX stuck-at" },
    { HW_FAULT_ALU_ADDER_CARRY, "Adder Carry Chain Break", "Class A Integer ALU", "Bridging fault causes carry drop" },
    { HW_FAULT_ALU_MUL_PARTIAL, "Multiplier Partial Product Error", "Class A Integer ALU", "Wallace tree stuck-at-0" },
    { HW_FAULT_DIV_QUOTIENT_OFF1, "SRT Divider Quotient Error", "Class B Integer Divider", "SRT selection circuit fault" },
    { HW_FAULT_DIV_HIGHREG_FLIP, "Divisor Register SRAM Error", "Class B Integer Divider", "SRAM soft error in register file" },
    { HW_FAULT_FPU_MANTISSA_FLIP, "FPU Register Mantissa Bit Flip", "Class C FPU/SIMD", "FPU ECC uncorrectable error" },
    { HW_FAULT_FPU_EXPONENT_FLIP, "FPU Register Exponent Flip", "Class C FPU/SIMD", "Exponent bit causing Inf" },
    { HW_FAULT_FPU_ROUNDING_MODE, "MXCSR Rounding Control Corruption", "Class C FPU/SIMD", "MXCSR.RC bit stuck-at" },
    { HW_FAULT_SIMD_SIGN_FLIP, "SIMD Subtraction Sign Inverter Fault", "Class C FPU/SIMD", "AVX VSUBPD sign inverter stuck-at" },
    { HW_FAULT_FPU_NAN_PROPAGATE, "FPU NaN Propagation Unit Fault", "Class C FPU/SIMD", "NaN not properly propagated" },
    { HW_FAULT_CACHE_L1_BITFLIP, "L1 Data Cache SRAM Soft Error", "Class D Cache Subsystem", "L1D SEU element corruption" },
    { HW_FAULT_CTRL_LOOP_COUNTER, "Loop Counter Register Bit Flip", "Class E Control Flow", "SEU causes loop terminating early" },
    { HW_FAULT_TEMP_HIGH, "CPU Temperature Above Limit", "Class F Simulated Sensors", "Assign simulated temperature above its upper limit" },
    { HW_FAULT_FREQ_LOW, "CPU Frequency Below Limit", "Class F Simulated Sensors", "Assign simulated frequency below its lower limit" },
};
#define FAULT_TABLE_LEN (sizeof(FAULT_TABLE) / sizeof(FAULT_TABLE[0]))

typedef struct {
    atomic_int inject_fault;
    /* 0: idle/acknowledged; 1: pending; 2: sensor check or CLI update owns it. */
    atomic_int inject_arm;
    atomic_int target_core;
    atomic_int running;
    int version;
    int num_cores;
    int core_enabled[MAX_CORES];
    atomic_int simulated_temperature_mc;
    atomic_int simulated_frequency_mhz;
} shared_state_t;

static inline int is_sensor_fault(hw_fault_type_t fault)
{
    return fault == HW_FAULT_TEMP_HIGH || fault == HW_FAULT_FREQ_LOW;
}

static inline int sensor_exceeds_threshold(hw_fault_type_t fault, int value)
{
    if (fault == HW_FAULT_TEMP_HIGH) return value > SIM_TEMP_LIMIT_MC;
    if (fault == HW_FAULT_FREQ_LOW) return value < SIM_FREQ_MIN_MHZ;
    return 0;
}

typedef struct {
    hw_fault_type_t fault;
    int core;
    int value;
} sensor_event_t;

/* Claim a consistent snapshot and keep it owned until the report is complete. */
static inline int claim_sensor_event(shared_state_t *state, sensor_event_t *event)
{
    if (!is_sensor_fault((hw_fault_type_t)atomic_load(&state->inject_fault))) return 0;
    int pending = 1;
    if (!atomic_compare_exchange_strong(&state->inject_arm, &pending, 2)) return 0;
    event->fault = (hw_fault_type_t)atomic_load(&state->inject_fault);
    event->core = atomic_load(&state->target_core);
    event->value = event->fault == HW_FAULT_TEMP_HIGH
        ? atomic_load(&state->simulated_temperature_mc)
        : atomic_load(&state->simulated_frequency_mhz);
    if (is_sensor_fault(event->fault) && event->core >= 0 && event->core < MAX_CORES &&
        state->core_enabled[event->core] && sensor_exceeds_threshold(event->fault, event->value))
        return 1;
    atomic_store(&state->inject_arm, 1);
    return 0;
}

#endif
