#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <fcntl.h>
#include "cpu_fault_shared.h"

#define COL_GRN "\033[1;32m"
#define COL_YLW "\033[1;33m"
#define COL_CYN "\033[1;36m"
#define COL_RST "\033[0m"

static shared_state_t *g_shm;

static int parse_number(const char *text, int *value)
{
    char *end;
    errno = 0;
    long number = strtol(text, &end, 10);
    if (errno || end == text || *end || number < INT_MIN || number > INT_MAX) return 0;
    *value = (int)number;
    return 1;
}

static void print_status(void)
{
    printf("Simulated temperature = %d mC, upper limit = %d mC\n",
           atomic_load(&g_shm->simulated_temperature_mc), SIM_TEMP_LIMIT_MC);
    printf("Simulated frequency = %d MHz, lower limit = %d MHz\n",
           atomic_load(&g_shm->simulated_frequency_mhz), SIM_FREQ_MIN_MHZ);
    printf("Target CPU = %d, fault = %d, pending = %d\n",
           atomic_load(&g_shm->target_core), atomic_load(&g_shm->inject_fault),
           atomic_load(&g_shm->inject_arm));
}

static void print_menu(void)
{
    printf("\nSelect CPU fault to inject\n");
    const char *last_category = "";
    for (size_t i = 0; i < FAULT_TABLE_LEN; i++) {
        if (strcmp(last_category, FAULT_TABLE[i].category)) {
            last_category = FAULT_TABLE[i].category;
            printf(COL_YLW "\n[%s]\n" COL_RST, last_category);
        }
        printf("  %2zu. %s\n", i + 1, FAULT_TABLE[i].short_name);
    }
    printf("\n   0. Clear injection and restore normal simulated values\n"
           "   s. Show simulated values and thresholds\n"
           "   q. Exit CLI\n\nChoice> ");
    fflush(stdout);
}

static int clear_injection(void)
{
    for (int i = 0; i < 1000; i++) {
        int state = atomic_load(&g_shm->inject_arm);
        if (state != 2 && atomic_compare_exchange_strong(&g_shm->inject_arm, &state, 2)) {
            atomic_store(&g_shm->inject_fault, HW_FAULT_NONE);
            atomic_store(&g_shm->target_core, -1);
            atomic_store(&g_shm->simulated_temperature_mc, SIM_TEMP_NORMAL_MC);
            atomic_store(&g_shm->simulated_frequency_mhz, SIM_FREQ_NORMAL_MHZ);
            atomic_store(&g_shm->inject_arm, 0);
            printf(COL_GRN "Injection cleared; simulated values reset.\n" COL_RST);
            printf("This does not bring an offline CPU back online.\n");
            return 0;
        }
        usleep(1000);
    }
    fprintf(stderr, "Detector/another injector is busy; try clearing again.\n");
    return 1;
}

static int core_available(int core, int sensor)
{
    if (core < 0 || core >= MAX_CORES || !g_shm->core_enabled[core]) return 0;
    if (sensor && core == 0) return 0;
    char path[128];
    snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/online", core);
    FILE *file = fopen(path, "r");
    if (!file) return !sensor && errno == ENOENT;
    int online = 0;
    int read_ok = fscanf(file, "%d", &online) == 1;
    fclose(file);
    return read_ok && online == 1;
}

static int inject(int choice, int target)
{
    if (choice < 1 || choice > (int)FAULT_TABLE_LEN) {
        fprintf(stderr, "Invalid fault selection.\n");
        return 1;
    }
    const fault_desc_t *fd = &FAULT_TABLE[choice - 1];
    int sensor = is_sensor_fault(fd->type);
    if (target == -1) {
        int candidates[MAX_CORES], count = 0;
        for (int i = 0; i < MAX_CORES; i++)
            if (core_available(i, sensor)) candidates[count++] = i;
        if (count == 0) {
            fprintf(stderr, "No online CPU is available for this fault.\n");
            return 1;
        }
        target = candidates[rand() % count];
    }
    if (!core_available(target, sensor)) {
        fprintf(stderr, "CPU %d is unavailable/offline. Sensor tests require an online nonzero CPU with an online interface.\n", target);
        return 1;
    }
    int idle = 0;
    if (!atomic_compare_exchange_strong(&g_shm->inject_arm, &idle, 2)) {
        fprintf(stderr, "An injection is pending. Wait for detection, or use 0 to clear it.\n");
        return 1;
    }
    atomic_store(&g_shm->simulated_temperature_mc, SIM_TEMP_NORMAL_MC);
    atomic_store(&g_shm->simulated_frequency_mhz, SIM_FREQ_NORMAL_MHZ);
    atomic_store(&g_shm->target_core, target);
    atomic_store(&g_shm->inject_fault, fd->type);
    if (fd->type == HW_FAULT_TEMP_HIGH)
        atomic_store(&g_shm->simulated_temperature_mc, SIM_TEMP_FAULT_MC);
    if (fd->type == HW_FAULT_FREQ_LOW)
        atomic_store(&g_shm->simulated_frequency_mhz, SIM_FREQ_FAULT_MHZ);
    printf(COL_CYN "\nInjected: %s; target CPU %d\n" COL_RST, fd->short_name, target);
    if (sensor) print_status();
    /* Publish only after target, type and synthetic values are consistent. */
    atomic_store(&g_shm->inject_arm, 1);

    for (int i = 0; i < 50; i++) {
        if (!atomic_load(&g_shm->running)) {
            fprintf(stderr, "Detector stopped before confirmation.\n");
            return 1;
        }
        if (atomic_load(&g_shm->inject_arm) == 0) {
            printf(COL_GRN "Detection acknowledged.\n" COL_RST);
            printf("Check monitor.sh output for the disposition result; acknowledgement alone is not interception success.\n");
            return 0;
        }
        usleep(100000);
    }
    fprintf(stderr, "Detection acknowledgement timed out after 5 seconds; injection remains pending.\n");
    return 2;
}

int main(int argc, char **argv)
{
    if (argc > 3) {
        fprintf(stderr, "Usage: %s [FAULT CORE | 0 | --status]\n", argv[0]);
        return 1;
    }
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0600);
    if (shm_fd < 0) {
        fprintf(stderr, "Cannot connect to detector. Start the rebuilt detector first.\n");
        return 1;
    }
    /* The daemon holds an exclusive lock until exit; an unlocked object is stale. */
    if (flock(shm_fd, LOCK_SH | LOCK_NB) == 0) {
        fprintf(stderr, "Shared memory is stale: no detector is running.\n");
        close(shm_fd);
        return 1;
    }
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
        perror("flock"); close(shm_fd); return 1;
    }
    struct stat st;
    if (fstat(shm_fd, &st) != 0 || st.st_size != (off_t)sizeof(shared_state_t)) {
        fprintf(stderr, "Shared memory size mismatch. Rebuild/restart both programs.\n");
        close(shm_fd); return 1;
    }
    g_shm = mmap(NULL, sizeof(*g_shm), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (g_shm == MAP_FAILED) { perror("mmap"); close(shm_fd); return 1; }
    int result = 0;
    if (!atomic_load(&g_shm->running) || g_shm->version != SHM_VERSION) {
        fprintf(stderr, "Detector is not ready or the protocol version differs.\n");
        result = 1;
        goto done;
    }
    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    if (argc > 1) {
        int choice, target;
        if (argc == 2 && !strcmp(argv[1], "--status")) print_status();
        else if (argc == 2 && !strcmp(argv[1], "0")) result = clear_injection();
        else if (argc == 3 && parse_number(argv[1], &choice) && parse_number(argv[2], &target))
            result = inject(choice, target);
        else {
            fprintf(stderr, "Usage: %s [FAULT CORE | 0 | --status]\n", argv[0]);
            result = 1;
        }
        goto done;
    }
    char input[128];
    while (atomic_load(&g_shm->running)) {
        print_menu();
        if (!fgets(input, sizeof(input), stdin)) break;
        input[strcspn(input, "\r\n")] = 0;
        if (!strcmp(input, "q") || !strcmp(input, "Q")) break;
        if (!strcmp(input, "s")) { print_status(); continue; }
        int choice, target;
        if (!parse_number(input, &choice) || choice < 0 || choice > (int)FAULT_TABLE_LEN) {
            fprintf(stderr, "Invalid option.\n"); continue;
        }
        if (choice == 0) { clear_injection(); continue; }
        printf("Physical CPU ID (-1 selects one available CPU at random)> ");
        fflush(stdout);
        if (!fgets(input, sizeof(input), stdin)) break;
        input[strcspn(input, "\r\n")] = 0;
        if (!parse_number(input, &target) || target < -1) {
            fprintf(stderr, "Invalid CPU ID.\n"); continue;
        }
        inject(choice, target);
    }
done:
    munmap(g_shm, sizeof(*g_shm));
    close(shm_fd);
    return result;
}
