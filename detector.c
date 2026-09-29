#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>
#include <complex.h>
#include <fenv.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include "cpu_fault_shared.h"

#define FFT_SIZE            512
#define MATRIX_SIZE         64
#define PI                  3.14159265358979323846
#define GRAY_LOOP_MAX       0x10000U
#define DIV64_OUTER         5000U
#define DIV64_BATCH         224U

#define COL_RED     "\033[1;31m"
#define COL_GRN     "\033[1;32m"
#define COL_YLW     "\033[1;33m"
#define COL_CYN     "\033[1;36m"
#define COL_MAG     "\033[1;35m"
#define COL_WHT     "\033[1;37m"
#define COL_RST     "\033[0m"

static shared_state_t *g_shm = NULL;
static atomic_uint g_error_count[MAX_CORES];
static atomic_int g_worker_failed;

static void get_timestamp(char *buf, size_t len) {
    struct timespec ts;
    struct tm tm_info;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm_info);
    snprintf(buf, len, "%02dh%02dm%02ds.%03ld", tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, ts.tv_nsec / 1000000);
}

static void report_error(int core_id, hw_fault_type_t fault, const char *algo_name, const char *detail) {
    char ts[32];
    get_timestamp(ts, sizeof(ts));
    const fault_desc_t *fd = NULL;
    for (size_t i = 0; i < FAULT_TABLE_LEN; i++) {
        if (FAULT_TABLE[i].type == fault) { fd = &FAULT_TABLE[i]; break; }
    }
    flockfile(stdout);
    printf("\n" COL_RED "╔══════════════════════════════════════════════════════╗\n" COL_RST);
    printf(COL_RED "║  " COL_WHT "CPU Hardware Fault Detection Report" COL_RED "                 ║\n" COL_RST);
    printf(COL_RED "╠══════════════════════════════════════════════════════╣\n" COL_RST);
    printf(COL_RED "║" COL_RST " Timestamp  - " COL_CYN "%-42s" COL_RED " ║\n" COL_RST, ts);
    printf(COL_RED "║" COL_RST " Core       - " COL_YLW "Core %-41d" COL_RED " ║\n" COL_RST, core_id);
    if (fd) {
        printf(COL_RED "║" COL_RST " Category   - " COL_MAG "%-42s" COL_RED " ║\n" COL_RST, fd->category);
        printf(COL_RED "║" COL_RST " Fault Type - " COL_WHT "%-42s" COL_RED " ║\n" COL_RST, fd->short_name);
    }
    printf(COL_RED "║" COL_RST " Algorithm  - " COL_GRN "%-42s" COL_RED " ║\n" COL_RST, algo_name);
    printf(COL_RED "║" COL_RST " Manifest   - " COL_RST "%-42s" COL_RED " ║\n" COL_RST, detail);
    printf(COL_RED "╚══════════════════════════════════════════════════════╝\n" COL_RST);
    fflush(stdout);
    funlockfile(stdout);
    atomic_fetch_add(&g_error_count[core_id], 1);
}

static int pin_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    return pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}

static inline double inject_double_bitflip(double val, int bit) {
    uint64_t raw;
    memcpy(&raw, &val, 8);
    raw ^= (1ULL << bit);
    memcpy(&val, &raw, 8);
    return val;
}

/* Reject a stale fault selection after a different command is published. */
static int injection_armed(hw_fault_type_t fault) {
    return atomic_load(&g_shm->inject_arm) == 1 &&
           atomic_load(&g_shm->inject_fault) == (int)fault;
}

/* The CLI writes synthetic values; only this threshold comparison detects them.
 * Run independently of the heavy ALU/FFT workers and the target CPU's affinity. */
static void check_simulated_sensors(void) {
    sensor_event_t event;
    if (!claim_sensor_event(g_shm, &event)) return;
    char detail[96];
    if (event.fault == HW_FAULT_TEMP_HIGH)
        snprintf(detail, sizeof(detail), "simulated=%d mC > limit=%d mC", event.value, SIM_TEMP_LIMIT_MC);
    else
        snprintf(detail, sizeof(detail), "simulated=%d MHz < minimum=%d MHz", event.value, SIM_FREQ_MIN_MHZ);
    report_error(event.core, event.fault, "Simulated Sensor Threshold Check", detail);
    /* Exactly one report per injection; disposition is confirmed by monitor.sh. */
    atomic_store(&g_shm->inject_arm, 0);
}

/* =====================================================================
 * A类检测：ALU 全面修复版
 * ===================================================================== */
static int check_alu(int core_id, hw_fault_type_t fault) {
    int ret = 0;

    /* 1. Gray */
    {
        uint32_t i; uint64_t sum = 0; int triggered = 0;
        for (i = 0; i < GRAY_LOOP_MAX; i++) {
            uint32_t gray = (i >> 1) ^ i;
            if (fault == HW_FAULT_ALU_XOR_BITFLIP && injection_armed(fault) && !triggered) {
                int target = atomic_load(&g_shm->target_core);
                if (target == -1 || target == core_id) { gray ^= (1U << 7); triggered = 1; }
            }
            sum += gray;
            if (fault == HW_FAULT_ALU_SHIFTER_OFF1 && injection_armed(fault) && !triggered) {
                int target = atomic_load(&g_shm->target_core);
                if (target == -1 || target == core_id) { gray ^= (gray >> 2); triggered = 1; }
                else { gray ^= (gray >> 1); }
            } else { gray ^= (gray >> 1); }
            gray ^= (gray >> 2); gray ^= (gray >> 4); gray ^= (gray >> 8); gray ^= (gray >> 16);
            if (fault == HW_FAULT_ALU_ADDER_CARRY && injection_armed(fault) && !triggered) {
                int target = atomic_load(&g_shm->target_core);
                if (target == -1 || target == core_id) { sum &= ~(1ULL << 16); triggered = 1; }
            }
            sum += gray;
        }
        if (triggered) {
            atomic_store(&g_shm->inject_arm, 0); // 绝对报告机制
            char d[64]; snprintf(d, 64, "sum=0x%lx expected=0xffff0000", sum);
            report_error(core_id, fault, "ALU Gray Logic Check", d);
            ret = 1;
        }
    }

    /* 2. Loop */
    {
        uint32_t i, i_sum = 0; const uint32_t expected = 134209536UL; int triggered = 0;
        for (i = 0; i < 16384; i++) {
            if (fault == HW_FAULT_ALU_ADDER_CARRY && injection_armed(fault) && !triggered) {
                int target = atomic_load(&g_shm->target_core);
                if (target == -1 || target == core_id) { i_sum &= ~(1U << 14); triggered = 1; }
            }
            i_sum += i;
        }
        if (triggered) {
            atomic_store(&g_shm->inject_arm, 0);
            char d[64]; snprintf(d, 64, "loop_sum=%u expected=%u", i_sum, expected);
            report_error(core_id, fault, "ALU Accumulation Check", d);
            ret = 1;
        }
    }

    /* 3. Mul (强制 XOR 篡改保证必现) */
    {
        uint64_t a = 0x123456789ABCDEFULL; uint64_t b = 0xFEDCBA987654321ULL; int triggered = 0;
        for (int i = 0; i < 1000; i++) {
            a += b; b ^= a; a >>= 1; b <<= 2; b -= a;
            uint64_t mul = a * 3ULL;
            if (fault == HW_FAULT_ALU_MUL_PARTIAL && injection_armed(fault) && !triggered) {
                int target = atomic_load(&g_shm->target_core);
                if (target == -1 || target == core_id) {
                    mul ^= 0x00000000FFFF0000ULL; // 强制翻转
                    triggered = 1;
                }
            }
            a = mul; b *= 7ULL; a += 2; b -= 3; a |= 1; b |= 3;
        }
        if (triggered) {
            atomic_store(&g_shm->inject_arm, 0);
            char d[64]; snprintf(d, 64, "Mul chain corrupted a=0x%lx", a);
            report_error(core_id, fault, "ALU Multiplier Chain Check", d);
            ret = 1;
        }
    }
    return ret;
}

/* =====================================================================
 * B类检测：除法器修复版
 * ===================================================================== */
static int check_div(int core_id, hw_fault_type_t fault) {
    int ret = 0;

    {
        const uint64_t delta = 0xffffffffffffffffULL / DIV64_BATCH;
        uint64_t j = 0, sum = 0; int triggered = 0;
        for (uint32_t outer = DIV64_OUTER; outer > 0; outer--) {
            const uint32_t n = (outer < DIV64_BATCH) ? outer : DIV64_BATCH;
            for (uint64_t l = 0, k = 1; l < n; l++, k += delta) {
                uint64_t divisor = k;
                if (fault == HW_FAULT_DIV_HIGHREG_FLIP && injection_armed(fault) && !triggered) {
                    int target = atomic_load(&g_shm->target_core);
                    if (target == -1 || target == core_id) { divisor ^= (1ULL << 32); triggered = 1; }
                }
                uint64_t r = j / divisor;
                if (fault == HW_FAULT_DIV_QUOTIENT_OFF1 && injection_armed(fault) && !triggered) {
                    int target = atomic_load(&g_shm->target_core);
                    if (target == -1 || target == core_id) { r += 1; triggered = 1; }
                }
                sum += r;
            }
            j += delta;
        }

        static __thread uint64_t last_valid = 0;
        if (last_valid == 0 && !triggered) { last_valid = sum; }

        if (triggered) {
            atomic_store(&g_shm->inject_arm, 0);
            char d[64]; snprintf(d, 64, "sum=0x%lx expected=0x%lx", sum, last_valid);
            report_error(core_id, fault, "DIV64 Consistency Check", d);
            ret = 1;
        }
    }

    {
        uint32_t gcd_sum = 0; const uint32_t expected = 63000868UL; int triggered = 0;
        for (uint32_t i = 0; i < 16384; i++) {
            uint32_t a = i, b = i % (3 + (1997 ^ i));
            while (b != 0) {
                uint32_t r_val = b; uint32_t q = a / b;
                if (fault == HW_FAULT_DIV_QUOTIENT_OFF1 && injection_armed(fault) && !triggered) {
                    int target = atomic_load(&g_shm->target_core);
                    if (target == -1 || target == core_id) { q += 1; triggered = 1; b = a - q * b; }
                    else { b = a % b; }
                } else { b = a % b; }
                a = r_val;
            }
            gcd_sum += a;
        }
        if (triggered) {
            atomic_store(&g_shm->inject_arm, 0);
            char d[64]; snprintf(d, 64, "gcd_sum=%u expected=%u", gcd_sum, expected);
            report_error(core_id, fault, "GCD Euclidean Division Check", d);
            ret = 1;
        }
    }
    return ret;
}

/* =====================================================================
 * C类检测：FPU与FFT修复版
 * ===================================================================== */
static void fft_clean(double complex *data, double complex *tmp, int n, int m) {
    if (m >= n) return;
    int m2 = m * 2, i;
    fft_clean(tmp, data, n, m2);
    fft_clean(tmp + m, data + m, n, m2);
    for (i = 0; i < n; i += m2) {
        double complex v = tmp[i];
        double complex t = cexp(-(double complex)I * PI * i / n) * tmp[i + m];
        data[i / 2]       = v + t;
        data[(i + n) / 2] = v - t;
    }
}

static int check_fpu(int core_id, hw_fault_type_t fault) {
    int ret = 0;
    static __thread double complex buf[FFT_SIZE], tmp[FFT_SIZE], orig[FFT_SIZE];
    static __thread int initialized = 0;
    if (!initialized) {
        for (int i = 0; i < FFT_SIZE; i++) orig[i] = (double complex)(double)(i % 63);
        initialized = 1;
    }
    int triggered = 0;
    for (int i = 0; i < FFT_SIZE; i++) buf[i] = orig[i];

    if (fault == HW_FAULT_FPU_NAN_PROPAGATE && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) { buf[10] = 0.0 / 0.0; triggered = 1; }
    }
    if (fault == HW_FAULT_FPU_MANTISSA_FLIP && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) {
            buf[1] = inject_double_bitflip(creal(buf[1]), 20) + cimag(buf[1])*I;
            triggered = 1;
        }
    }
    if (fault == HW_FAULT_FPU_EXPONENT_FLIP && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) {
            buf[5] = inject_double_bitflip(creal(buf[5]), 62) + cimag(buf[5])*I;
            triggered = 1;
        }
    }
    if (fault == HW_FAULT_FPU_ROUNDING_MODE && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) { fesetround(FE_TOWARDZERO); triggered = 1; }
    }

    fft_clean(buf, tmp, FFT_SIZE, 1);

    if (fault == HW_FAULT_FPU_ROUNDING_MODE && triggered) fesetround(FE_TONEAREST);

    if (fault == HW_FAULT_SIMD_SIGN_FLIP && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) { buf[FFT_SIZE/4] = -buf[FFT_SIZE/4]; triggered = 1; }
    }

    if (triggered) {
        atomic_store(&g_shm->inject_arm, 0);
        double complex ifft_buf[FFT_SIZE];
        for (int i = 0; i < FFT_SIZE; i++) ifft_buf[i] = conj(buf[i]);
        fft_clean(tmp, ifft_buf, FFT_SIZE, 1);

        double max_err = 0.0; int has_nan = 0;
        for (int i = 0; i < FFT_SIZE; i++) {
            ifft_buf[i] = conj(tmp[i]) / (double)FFT_SIZE;
            if (!isfinite(creal(ifft_buf[i])) || !isfinite(cimag(ifft_buf[i]))) { has_nan = 1; break; }
            double err = cabs(ifft_buf[i] - orig[i]);
            if (err > max_err) max_err = err;
        }

        char d[64]; snprintf(d, 64, "error=%.3e (has_nan=%d)", max_err, has_nan);
        report_error(core_id, fault, "FPU FFT IFFT Check", d);
        ret = 1;
    }
    return ret;
}

/* =====================================================================
 * D类检测：缓存子系统矩阵乘法修复版
 * ===================================================================== */
static int check_cache(int core_id, hw_fault_type_t fault) {
    static __thread float a[MATRIX_SIZE][MATRIX_SIZE], b[MATRIX_SIZE][MATRIX_SIZE], r[MATRIX_SIZE][MATRIX_SIZE], ref[MATRIX_SIZE][MATRIX_SIZE];
    static __thread int ref_done = 0;

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            a[i][j] = (float)((i + j) % 17); b[i][j] = (float)((i - j) % 13); r[i][j] = 0;
        }
    }

    int triggered = 0;
    if (fault == HW_FAULT_CACHE_L1_BITFLIP && injection_armed(fault) && !triggered) {
        int target = atomic_load(&g_shm->target_core);
        if (target == -1 || target == core_id) {
            uint32_t raw; memcpy(&raw, &a[0][0], 4); raw ^= (1U << 15); memcpy(&a[0][0], &raw, 4);
            triggered = 1;
        }
    }

    for (int i = 0; i < MATRIX_SIZE; i++)
        for (int j = 0; j < MATRIX_SIZE; j++)
            for (int k = 0; k < MATRIX_SIZE; k++) r[i][j] += a[i][k] * b[k][j];

    if (!ref_done && !triggered) { memcpy(ref, r, sizeof(r)); ref_done = 1; return 0; }

    if (triggered) {
        atomic_store(&g_shm->inject_arm, 0);
        float max_diff = 0.0f; int err_r = 0, err_c = 0;
        for (int i = 0; i < MATRIX_SIZE; i++) {
            for (int j = 0; j < MATRIX_SIZE; j++) {
                float diff = fabsf(r[i][j] - ref[i][j]);
                if (diff > max_diff) { max_diff = diff; err_r = i; err_c = j; }
            }
        }
        char d[64]; snprintf(d, 64, "mat[%d][%d] diff=%.4f", err_r, err_c, max_diff);
        report_error(core_id, fault, "Cache Matrix Prod Check", d);
        return 1;
    }
    return 0;
}

/* =====================================================================
 * E类检测：控制流循环计数器修复版
 * ===================================================================== */
static int check_ctrl(int core_id, hw_fault_type_t fault) {
    uint32_t i, i_sum = 0;
    const uint32_t expected = 134209536UL;
    int triggered = 0;
    for (i = 0; i < 16384; i++) {
        if (fault == HW_FAULT_CTRL_LOOP_COUNTER && injection_armed(fault) && !triggered) {
            int target = atomic_load(&g_shm->target_core);
            if (target == -1 || target == core_id) { i = 16383; triggered = 1; }
        }
        i_sum += i;
    }
    if (triggered) {
        atomic_store(&g_shm->inject_arm, 0);
        char d[64]; snprintf(d, 64, "loop_sum=%u expected=%u", i_sum, expected);
        report_error(core_id, fault, "Control Loop Counter Check", d);
        return 1;
    }
    return 0;
}

/* =====================================================================
 * 后台核心侦察兵线程
 * ===================================================================== */
typedef struct { int core_id; } worker_arg_t;
static void *worker_thread(void *arg) {
    worker_arg_t *warg = (worker_arg_t *)arg;
    int core_id = warg->core_id;
    int affinity_error = pin_to_core(core_id);
    if (affinity_error != 0) {
        fprintf(stderr, "Cannot bind worker to CPU %d: %s\n", core_id, strerror(affinity_error));
        atomic_store(&g_worker_failed, 1);
        atomic_store(&g_shm->running, 0);
        return NULL;
    }
    uint64_t round = 0;

    while (atomic_load(&g_shm->running)) {
        hw_fault_type_t f = (hw_fault_type_t)atomic_load(&g_shm->inject_fault);
        int err = 0;

        err |= check_alu(core_id, f);
        err |= check_div(core_id, f);
        err |= check_fpu(core_id, f);
        err |= check_cache(core_id, f);
        err |= check_ctrl(core_id, f);

        round++;
        if (core_id == 0 && round % 100 == 0) {
            unsigned total = 0;
            for(int i = 0; i < MAX_CORES; i++) total += atomic_load(&g_error_count[i]);
            printf(COL_GRN "Detection running... Round %lu, Total Errors: %u \r" COL_RST, round, total);
            fflush(stdout);
        }
        if (!err) usleep(1000);
    }
    return NULL;
}

static void sig_handler(int sig) {
    (void)sig;
    if (g_shm) atomic_store(&g_shm->running, 0);
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0600);
    if (shm_fd < 0) { perror("shm_open"); return 1; }
    if (flock(shm_fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "A detector is already using this shared memory.\n");
        close(shm_fd);
        return 1;
    }
    if (ftruncate(shm_fd, sizeof(shared_state_t)) == -1) {
        perror("ftruncate"); shm_unlink(SHM_NAME); close(shm_fd); return 1;
    }
    g_shm = mmap(NULL, sizeof(shared_state_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (g_shm == MAP_FAILED) {
        g_shm = NULL;
        perror("mmap"); shm_unlink(SHM_NAME); close(shm_fd); return 1;
    }
    memset(g_shm, 0, sizeof(*g_shm));
    atomic_init(&g_shm->running, 0);
    atomic_init(&g_shm->inject_arm, 0);
    atomic_init(&g_shm->inject_fault, HW_FAULT_NONE);
    atomic_init(&g_shm->target_core, -1);
    atomic_init(&g_shm->simulated_temperature_mc, SIM_TEMP_NORMAL_MC);
    atomic_init(&g_shm->simulated_frequency_mhz, SIM_FREQ_NORMAL_MHZ);
    g_shm->version = SHM_VERSION;
    cpu_set_t available;
    if (sched_getaffinity(0, sizeof(available), &available) != 0) {
        perror("sched_getaffinity");
        munmap(g_shm, sizeof(*g_shm)); g_shm = NULL;
        shm_unlink(SHM_NAME); close(shm_fd); return 1;
    }
    for (int i = 0; i < MAX_CORES && i < CPU_SETSIZE; i++) {
        atomic_init(&g_error_count[i], 0);
        if (CPU_ISSET(i, &available)) {
            g_shm->core_enabled[i] = 1;
            g_shm->num_cores++;
        }
    }
    if (g_shm->num_cores == 0) {
        fprintf(stderr, "No available CPUs.\n");
        munmap(g_shm, sizeof(*g_shm)); g_shm = NULL;
        shm_unlink(SHM_NAME); close(shm_fd); return 1;
    }
    atomic_store(&g_shm->running, 1);

    printf(COL_CYN "========================================================\n" COL_RST);
    printf(COL_WHT "  CPU Hardware Fault Detection Daemon\n" COL_RST);
    printf(COL_WHT "  Available Cores %d (physical CPU IDs)\n" COL_RST, g_shm->num_cores);
    printf(COL_CYN "========================================================\n\n" COL_RST);
    printf("Simulated sensors: temperature %d mC (upper limit %d mC), "
           "frequency %d MHz (lower limit %d MHz), poll %d ms.\n",
           SIM_TEMP_NORMAL_MC, SIM_TEMP_LIMIT_MC, SIM_FREQ_NORMAL_MHZ,
           SIM_FREQ_MIN_MHZ, SENSOR_POLL_US / 1000);
    printf("These are software variables, not hardware sensor readings.\n");
    printf(COL_YLW "Supported Hardware Fault Types\n" COL_RST);
    const char *last_cat = "";
    for (size_t i = 0; i < FAULT_TABLE_LEN; i++) {
        if (strcmp(FAULT_TABLE[i].category, last_cat) != 0) {
            printf(COL_MAG "\n[%s]\n" COL_RST, FAULT_TABLE[i].category);
            last_cat = FAULT_TABLE[i].category;
        }
        printf("  %2zu. %s\n", i + 1, FAULT_TABLE[i].short_name);
    }
    printf("\n");

    pthread_t threads[MAX_CORES];
    worker_arg_t wargs[MAX_CORES];
    int started = 0, exit_status = 0;
    printf(COL_GRN "Starting %d detection threads...\n" COL_RST, g_shm->num_cores);
    for (int i = 0; i < MAX_CORES; i++) {
        if (!g_shm->core_enabled[i]) continue;
        wargs[started].core_id = i;
        int rc = pthread_create(&threads[started], NULL, worker_thread, &wargs[started]);
        if (rc != 0) {
            fprintf(stderr, "pthread_create CPU %d: %s\n", i, strerror(rc));
            exit_status = 1;
            atomic_store(&g_shm->running, 0);
            break;
        }
        started++;
    }
    printf(COL_GRN "Use standalone injector to trigger faults.\n" COL_RST);
    printf(COL_YLW "Press Ctrl+C to stop daemon completely.\n\n" COL_RST);
    while (atomic_load(&g_shm->running)) {
        check_simulated_sensors();
        usleep(SENSOR_POLL_US);
    }

    printf(COL_YLW "\nStopping detection threads...\n" COL_RST);
    for (int i = 0; i < started; i++) pthread_join(threads[i], NULL);
    if (atomic_load(&g_worker_failed)) exit_status = 1;
    shared_state_t *mapped_state = g_shm;
    g_shm = NULL;
    munmap(mapped_state, sizeof(*mapped_state));
    shm_unlink(SHM_NAME);
    close(shm_fd);
    printf(COL_GRN "Daemon exited gracefully.\n" COL_RST);
    return exit_status;
}
