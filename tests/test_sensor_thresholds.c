#include <assert.h>
#include <stdio.h>
#include "../cpu_fault_shared.h"

int main(void)
{
    assert(HW_FAULT_CTRL_LOOP_COUNTER == 13);
    assert(HW_FAULT_TEMP_HIGH == 14);
    assert(HW_FAULT_FREQ_LOW == 15);
    assert(FAULT_TABLE_LEN == 15);
    assert(MAX_CORES >= 128);
    for (size_t i = 0; i < FAULT_TABLE_LEN; i++)
        assert(FAULT_TABLE[i].type == (hw_fault_type_t)(i + 1));

    assert(!sensor_exceeds_threshold(HW_FAULT_TEMP_HIGH, SIM_TEMP_NORMAL_MC));
    assert(!sensor_exceeds_threshold(HW_FAULT_TEMP_HIGH, SIM_TEMP_LIMIT_MC - 1));
    assert(!sensor_exceeds_threshold(HW_FAULT_TEMP_HIGH, SIM_TEMP_LIMIT_MC));
    assert(sensor_exceeds_threshold(HW_FAULT_TEMP_HIGH, SIM_TEMP_LIMIT_MC + 1));
    assert(sensor_exceeds_threshold(HW_FAULT_TEMP_HIGH, SIM_TEMP_FAULT_MC));
    assert(!sensor_exceeds_threshold(HW_FAULT_FREQ_LOW, SIM_FREQ_NORMAL_MHZ));
    assert(!sensor_exceeds_threshold(HW_FAULT_FREQ_LOW, SIM_FREQ_MIN_MHZ + 1));
    assert(!sensor_exceeds_threshold(HW_FAULT_FREQ_LOW, SIM_FREQ_MIN_MHZ));
    assert(sensor_exceeds_threshold(HW_FAULT_FREQ_LOW, SIM_FREQ_MIN_MHZ - 1));
    assert(sensor_exceeds_threshold(HW_FAULT_FREQ_LOW, SIM_FREQ_FAULT_MHZ));
    assert(!sensor_exceeds_threshold(HW_FAULT_ALU_XOR_BITFLIP, SIM_TEMP_FAULT_MC));
    shared_state_t state = {0};
    atomic_init(&state.inject_arm, 0);
    atomic_init(&state.inject_fault, HW_FAULT_TEMP_HIGH);
    atomic_init(&state.target_core, 127);
    atomic_init(&state.simulated_temperature_mc, SIM_TEMP_NORMAL_MC);
    atomic_init(&state.simulated_frequency_mhz, SIM_FREQ_NORMAL_MHZ);
    state.core_enabled[127] = 1;
    sensor_event_t event;
    atomic_store(&state.inject_arm, 1);
    assert(!claim_sensor_event(&state, &event));
    atomic_store(&state.simulated_temperature_mc, SIM_TEMP_LIMIT_MC);
    assert(!claim_sensor_event(&state, &event));
    atomic_store(&state.inject_fault, HW_FAULT_FREQ_LOW);
    atomic_store(&state.simulated_frequency_mhz, SIM_FREQ_MIN_MHZ);
    assert(!claim_sensor_event(&state, &event));
    assert(atomic_load(&state.inject_arm) == 1);

    for (int fault = HW_FAULT_TEMP_HIGH; fault <= HW_FAULT_FREQ_LOW; fault++) {
        for (int i = 0; i < 5; i++) {
            atomic_store(&state.inject_arm, 2);
            atomic_store(&state.inject_fault, fault);
            atomic_store(&state.simulated_temperature_mc,
                         fault == HW_FAULT_TEMP_HIGH ? SIM_TEMP_FAULT_MC : SIM_TEMP_NORMAL_MC);
            atomic_store(&state.simulated_frequency_mhz,
                         fault == HW_FAULT_FREQ_LOW ? SIM_FREQ_FAULT_MHZ : SIM_FREQ_NORMAL_MHZ);
            assert(!claim_sensor_event(&state, &event));
            atomic_store(&state.inject_arm, 1);
            assert(claim_sensor_event(&state, &event));
            assert(event.core == 127 && event.fault == (hw_fault_type_t)fault);
            assert(event.value == (fault == HW_FAULT_TEMP_HIGH ? SIM_TEMP_FAULT_MC : SIM_FREQ_FAULT_MHZ));
            assert(atomic_load(&state.inject_arm) == 2);
            assert(!claim_sensor_event(&state, &event));
            atomic_store(&state.inject_arm, 0);
            assert(!claim_sensor_event(&state, &event));
        }
    }
    atomic_store(&state.target_core, MAX_CORES);
    atomic_store(&state.inject_arm, 1);
    assert(!claim_sensor_event(&state, &event));
    atomic_store(&state.target_core, -1);
    assert(!claim_sensor_event(&state, &event));
    atomic_store(&state.target_core, 126);
    assert(!claim_sensor_event(&state, &event));
    atomic_store(&state.target_core, 127);
    atomic_store(&state.inject_fault, HW_FAULT_CTRL_LOOP_COUNTER);
    assert(!claim_sensor_event(&state, &event));
    puts("PASS: IDs, threshold boundaries, 5+5 sensor events on CPU 127, no duplicates and invalid/busy states");
    return 0;
}
