// End-to-end tests on a simulated board: the real firmware modules (exc-runner, sync-demod,
// exc-plan, bridge) driven tick by tick exactly as tim2_tick does, against a physical model
// of the chain:
//   DAC (12 bit, optional sag under the bridge load, written ~25 us after the tick)
//   -> quarter bridge (R8 = 350R over the gauge, other half at Vexc/2)
//   -> AD8553, gain 100, REF 0.95 V, first-order output pole
//   -> LTC1865L (16 bit, 3.29 V, clipped), sampled at the tick, with noise, mains and
//      inverter interference.
#include <unity.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bridge-api.h"
#include "exc-plan-api.h"
#include "exc-runner-api.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TIM_CLK_HZ 72000000.0
#define ADC_LSB_V (3.29 / 65536.0)
#define DAC_V_PER_CODE (3.3 / 4095.0)
#define R_NOM 350.0
#define GF 2.1 // gauge factor of the electronics' calculator
// AD8553 output pole, if the 680 pF of the schematic sits across the 100k of the gain:
// 100k * 680p = 68 us (fc ~2.3 kHz). To be checked on the board.
#define AMP_TAU_S 68e-6
#define DAC_WRITE_DELAY_S 25e-6 // CONV 10 us + 16 SPI clocks at 1.125 MHz before the DAC write

// Same configuration as main.c
static const struct ExcRunnerConfig CFG = {
    .plan = {
        .tim_clk_hz = 72000000U,
        .max_samples = 64U,
        .min_samples = 16U,
        .max_tick_hz = 16000U,
        .max_periods = SYNC_DEMOD_MAX_PERIODS,
        .min_window_ms = 40U,
        .max_window_ms = 400U,
        .output_ms = 8U,
        .mains_hz = 50.0F,
        .mains_tol_hz = 0.1F,
        .mains_guard_hz = 5.0F,
    },
    .vdda_v = 3.3F,
    .out_min_v = 0.2F,
    .out_max_v = 3.1F,
    .dac_max = 4095U,
    .default_freq_hz = 125U,
    .default_dc_mv = 550U,
    .default_amp_mv = 300U,
};

static const struct BridgeConfig BRIDGE = {
    .amp_gain = 100.0F,
    .adc_lsb_v = (float)ADC_LSB_V,
    .cal_phase_rad = 0.0F,
    .clip_low_v = 0.02F,
    .clip_high_v = 3.25F,
};

struct Board {
    // physics
    double r_gauge;
    double gain;
    double ref_v;
    double tau_s;       // amplifier output pole (0 = ideal)
    double dac_delay_s; // DAC written this long after the tick (the ADC samples at the tick)
    double dac_gain;    // real DAC output over the ideal one (sag under load)
    double noise_lsb;   // ADC noise, rms counts
    double mains_v;     // mains at the amplifier output, amplitude
    double mains_hz;
    double inv_v;       // inverter tone at the ADC input, amplitude
    double inv_hz;
    // faults injected on the next tick
    bool spi_fail_next;
    bool garbage_next;
    // state
    double t;
    double v_amp;
    double vexc;
    uint32_t reload;
    uint32_t next_reload;
    uint64_t rng;
};

static struct ExcRunner runner;

void setUp(void) {}
void tearDown(void) {}

static void board_defaults(struct Board *b, double r_gauge) {
    memset(b, 0, sizeof(*b));
    b->r_gauge = r_gauge;
    b->gain = 100.0;
    b->ref_v = 0.95;
    b->tau_s = AMP_TAU_S;
    b->dac_delay_s = DAC_WRITE_DELAY_S;
    b->dac_gain = 1.0;
    b->noise_lsb = 1.5;
    b->mains_hz = 50.0;
    b->rng = 0x9E3779B97F4A7C15ULL;
}

static double gauss(struct Board *b) {
    // xorshift64* + Box-Muller: deterministic
    double u[2];
    for (int k = 0; k < 2; k++) {
        b->rng ^= b->rng >> 12;
        b->rng ^= b->rng << 25;
        b->rng ^= b->rng >> 27;
        u[k] = ((double)((b->rng * 2685821657736338717ULL) >> 11) + 0.5) / 9007199254740992.0;
    }
    return sqrt(-2.0 * log(u[0])) * cos(2.0 * M_PI * u[1]);
}

// Bridge imbalance (V+ - V-) / Vexc
static double imbalance(double r_gauge) {
    return 0.5 - r_gauge / (R_NOM + r_gauge);
}

static double amp_target(const struct Board *b, double vexc) {
    return b->ref_v + b->gain * vexc * imbalance(b->r_gauge);
}

static void settle(struct Board *b, double vexc, double dt) {
    double target = amp_target(b, vexc);
    b->v_amp = b->tau_s > 0.0 ? target + (b->v_amp - target) * exp(-dt / b->tau_s) : target;
}

static void board_start(struct Board *b) {
    TEST_ASSERT_TRUE(exc_runner_api_init(&runner, &CFG));
    b->vexc = runner.active->dac[0] * DAC_V_PER_CODE * b->dac_gain;
    b->v_amp = amp_target(b, b->vexc);
    b->reload = runner.active->plan.autoreload + 1U;
    b->next_reload = b->reload;
}

// One timer tick, as tim2_tick in main.c: sample, advance, write the DAC, demodulate.
static bool board_tick(struct Board *b, struct LockinResult *res, struct ExcStep *step_out) {
    double v = b->v_amp + b->noise_lsb * ADC_LSB_V * gauss(b);
    v += b->mains_v * sin(2.0 * M_PI * b->mains_hz * b->t + 0.3);
    v += b->inv_v * sin(2.0 * M_PI * b->inv_hz * b->t + 1.1);
    long code = lround(v / ADC_LSB_V);
    code = code < 0 ? 0 : (code > 65535 ? 65535 : code);
    if (b->garbage_next) {
        code = 65535; // e.g. the other channel, before the first channel selection
        b->garbage_next = false;
    }
    bool ok = !b->spi_fail_next;
    b->spi_fail_next = false;

    struct ExcStep step;
    exc_runner_api_advance(&runner, &step);
    if (step.switched) {
        b->next_reload = step.autoreload + 1U; // preload: from the next period
    }
    bool done = exc_runner_api_sample(&runner, &step, ok, (uint16_t)code, res);

    double dt = (double)b->reload / TIM_CLK_HZ;
    double new_vexc = step.dac * DAC_V_PER_CODE * b->dac_gain;
    settle(b, b->vexc, b->dac_delay_s); // old value until the ISR writes the DAC
    settle(b, new_vexc, dt - b->dac_delay_s);
    b->vexc = new_vexc;
    b->t += dt;
    b->reload = b->next_reload;
    if (step_out != NULL) {
        *step_out = step;
    }
    return done;
}

// Run until `n` results, converted; returns how many arrived within max_ticks.
static int board_run(struct Board *b, int n, struct BridgeReading *out, struct LockinResult *res_out,
                     uint32_t max_ticks) {
    int got = 0;
    for (uint32_t k = 0; k < max_ticks && got < n; k++) {
        struct LockinResult res;
        if (board_tick(b, &res, NULL)) {
            if (out != NULL) {
                bridge_api_convert(&BRIDGE, &res, &out[got]);
            }
            if (res_out != NULL) {
                res_out[got] = res;
            }
            got++;
        }
    }
    return got;
}

static double mean_out(const struct BridgeReading *r, int n) {
    double s = 0.0;
    for (int k = 0; k < n; k++) {
        s += r[k].out_mv_per_v;
    }
    return s / n;
}

static double std_out(const struct BridgeReading *r, int n) {
    double m = mean_out(r, n);
    double s = 0.0;
    for (int k = 0; k < n; k++) {
        s += (r[k].out_mv_per_v - m) * (r[k].out_mv_per_v - m);
    }
    return sqrt(s / (n - 1));
}

// mV/V -> microstrain, quarter bridge
static double strain_ue(double out_mv_per_v) {
    return -4.0 * out_mv_per_v / 1000.0 / GF * 1e6;
}

// Response of the sampled chain to the excitation, for the in-phase reading: one-pole
// amplifier, DAC written dac_delay after the tick, sample at the tick tagged with the DAC
// value written at the previous one.
static void chain_response(double tick_s, uint32_t samples, double tau, double delay, double *re, double *im) {
    double a1 = exp(-delay / tau);
    double a2 = exp(-(tick_s - delay) / tau);
    double th = 2.0 * M_PI / samples;
    // H = ((1 - a2) + a2 (1 - a1) e^-jth) / (1 - a1 a2 e^-jth)
    double nr = (1.0 - a2) + a2 * (1.0 - a1) * cos(th);
    double ni = -a2 * (1.0 - a1) * sin(th);
    double dr = 1.0 - a1 * a2 * cos(th);
    double di = a1 * a2 * sin(th);
    double den = dr * dr + di * di;
    *re = (nr * dr + ni * di) / den;
    *im = (ni * dr - nr * di) / den;
}

// ---------------------------------------------------------------------------------------
// The electronics' calculator: same gauge resistances, expected strain (GF 2.1), at the
// default 125 Hz with ADC noise and a realistic amplifier. Zero taken at Re = R.
void test_chain_electronics_expected_values(void) {
    static const double RE[] = { 350.004079, 350.014278, 350.016929, 350.026516, 350.046915 };
    static const double EXPECTED_UE[] = { 5.55004, 19.4254, 23.0331, 36.0764, 63.8293 };
    struct BridgeReading r[20];
    struct Board b;

    board_defaults(&b, R_NOM);
    board_start(&b);
    TEST_ASSERT_EQUAL_INT(20, board_run(&b, 20, r, NULL, 200000U));
    double zero = mean_out(r, 20);

    for (uint32_t k = 0; k < 5U; k++) {
        b.r_gauge = RE[k];
        board_run(&b, 10, r, NULL, 200000U); // the sliding window forgets the previous value
        TEST_ASSERT_EQUAL_INT(20, board_run(&b, 20, r, NULL, 200000U));
        double ue = strain_ue(mean_out(r, 20) - zero);
        char msg[96];
        snprintf(msg, sizeof(msg), "Re %.6f: %.2f ue misurati, %.2f attesi", RE[k], ue, EXPECTED_UE[k]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE((float)(0.5 + 0.01 * EXPECTED_UE[k]), (float)EXPECTED_UE[k], (float)ue, msg);
        TEST_ASSERT_EQUAL_UINT8(0, r[19].flags);
    }
}

// Large imbalance: the reading follows the exact bridge law within the ADC range, and the
// saturation is flagged outside it (lower side first: REF sits at 0.95 V).
void test_chain_large_imbalance_and_clipping(void) {
    static const double RE[] = { 336.0, 343.0, 357.0, 364.0 };
    struct BridgeReading r[4];
    for (uint32_t k = 0; k < 4U; k++) {
        struct Board b;
        board_defaults(&b, RE[k]);
        board_start(&b);
        TEST_ASSERT_EQUAL_INT(4, board_run(&b, 4, r, NULL, 200000U));
        double expected = 1000.0 * imbalance(RE[k]);
        TEST_ASSERT_FLOAT_WITHIN((float)(0.005 * fabs(expected)), (float)expected, r[3].out_mv_per_v);
        TEST_ASSERT_EQUAL_UINT8(0, r[3].flags);
    }

    struct Board b;
    board_defaults(&b, 380.0);
    board_start(&b);
    board_run(&b, 2, r, NULL, 200000U);
    TEST_ASSERT_BITS_HIGH(BRIDGE_FLAG_CLIP_LOW, r[1].flags);

    board_defaults(&b, 300.0);
    board_start(&b);
    board_run(&b, 2, r, NULL, 200000U);
    TEST_ASSERT_BITS_HIGH(BRIDGE_FLAG_CLIP_HIGH, r[1].flags);
}

// Every accepted frequency, ideal analog chain: the reading is the bridge output whatever
// the samples per period, timer rounding and window (the phase tagging holds everywhere).
void test_chain_all_frequencies_ideal_analog(void) {
    double expected = 1000.0 * imbalance(345.0);
    for (uint32_t f = 1U; f <= 1000U; f += 3U) {
        struct ExcPlan plan;
        if (!exc_plan_api_make(&CFG.plan, f, &plan)) {
            continue;
        }
        struct Board b;
        struct BridgeReading r[2];
        board_defaults(&b, 345.0);
        b.tau_s = 0.0;
        board_start(&b);
        TEST_ASSERT_TRUE(exc_runner_api_request(&runner, f, 0U, 0U));
        // ids: 0 default, 1 the requested frequency
        struct LockinResult res[8];
        int got = board_run(&b, 8, NULL, res, 400000U);
        TEST_ASSERT_GREATER_OR_EQUAL_INT(3, got);
        TEST_ASSERT_EQUAL_UINT8(1, res[got - 1].exc_id);
        bridge_api_convert(&BRIDGE, &res[got - 1], &r[0]);
        char msg[48];
        snprintf(msg, sizeof(msg), "f=%u Hz", (unsigned)f);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE((float)(0.002 * fabs(expected)), (float)expected, r[0].out_mv_per_v, msg);
    }
}

// Same with the amplifier pole and the DAC write delay: the in-phase reading is scaled by
// the real part of the chain response, which grows with the tick rate. The model above
// predicts it for every plan; the reported phase is its argument (usable to calibrate).
void test_chain_all_frequencies_amplifier_pole(void) {
    double expected = 1000.0 * imbalance(345.0);
    double worst = 1.0;
    uint32_t worst_f = 0;
    for (uint32_t f = 1U; f <= 1000U; f += 3U) {
        struct ExcPlan plan;
        if (!exc_plan_api_make(&CFG.plan, f, &plan)) {
            continue;
        }
        struct Board b;
        struct BridgeReading r[1];
        struct LockinResult res[8];
        board_defaults(&b, 345.0);
        board_start(&b);
        TEST_ASSERT_TRUE(exc_runner_api_request(&runner, f, 0U, 0U));
        int got = board_run(&b, 8, NULL, res, 400000U);
        TEST_ASSERT_GREATER_OR_EQUAL_INT(3, got);
        TEST_ASSERT_EQUAL_UINT8(1, res[got - 1].exc_id);
        bridge_api_convert(&BRIDGE, &res[got - 1], &r[0]);

        double hr;
        double hi;
        chain_response((plan.autoreload + 1.0) / TIM_CLK_HZ, plan.samples, AMP_TAU_S, DAC_WRITE_DELAY_S, &hr, &hi);
        double ratio = r[0].out_mv_per_v / expected;
        char msg[96];
        snprintf(msg, sizeof(msg), "f=%u Hz, %u campioni: letto %.4f, modello %.4f", (unsigned)f,
                 (unsigned)plan.samples, ratio, hr);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.003F, (float)hr, (float)ratio, msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5F, (float)(atan2(hi, hr) * 180.0 / M_PI), r[0].phase_deg, msg);
        if (ratio < worst) {
            worst = ratio;
            worst_f = f;
        }
    }
    printf("INFO polo dell'amplificatore: lettura al minimo %.1f%% del vero a %u Hz\n", 100.0 * worst,
           (unsigned)worst_f);
    // at the default 125 Hz the pole and the DAC write delay cost about 0.2%
    double hr;
    double hi;
    chain_response(1.0 / 8000.0, 64U, AMP_TAU_S, DAC_WRITE_DELAY_S, &hr, &hi);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.9982F, (float)hr);
}

// Bench mains at the amplifier output (10 mV, i.e. 0.1 mV at the bridge): exactly at 50 Hz
// it cancels, with the grid 0.1 Hz off it leaks as the model in exc-plan.h says.
void test_chain_mains_at_default_frequency(void) {
    static const double MAINS_HZ[] = { 49.9, 50.0, 50.1 };
    struct BridgeReading r[30];
    struct Board b;
    board_defaults(&b, R_NOM);
    board_start(&b);
    board_run(&b, 30, r, NULL, 400000U);
    double clean = mean_out(r, 30);

    for (uint32_t m = 0; m < 3U; m++) {
        board_defaults(&b, R_NOM);
        b.mains_v = 0.010;
        b.mains_hz = MAINS_HZ[m];
        board_start(&b);
        board_run(&b, 30, r, NULL, 400000U);
        double err_ue = fabs(strain_ue(mean_out(r, 30) - clean));
        // leak of the fundamental: d / |f - 50| + d / (f + 50) of its amplitude
        double d = fabs(MAINS_HZ[m] - 50.0);
        double leak_v = b.mains_v * (d / 75.0 + d / 175.0);
        double allowed_ue = 1.5 * fabs(strain_ue(1000.0 * leak_v / (100.0 * 0.3))) + 0.3;
        char msg[80];
        snprintf(msg, sizeof(msg), "rete %.1f Hz: %.2f ue, attesi <= %.2f", MAINS_HZ[m], err_ue, allowed_ue);
        TEST_ASSERT_TRUE_MESSAGE(err_ue <= allowed_ue, msg);
        printf("INFO rete 10 mV a %.1f Hz: errore %.2f ue\n", MAINS_HZ[m], err_ue);
    }
}

// Inverter PWM tone at the ADC (20 mV): sampled at the tick it aliases. Far from the
// excitation it is rejected; aliased 3 Hz away it beats in the reading and shows up as
// noise, which is what the sweep is meant to find.
void test_chain_inverter_alias(void) {
    struct BridgeReading r[150];
    struct Board b;

    // 125 Hz: tick 8 kHz, 14172 Hz aliases to 1828 Hz
    board_defaults(&b, R_NOM);
    board_start(&b);
    board_run(&b, 120, r, NULL, 400000U);
    double floor_ue = strain_ue(std_out(r, 120));
    board_defaults(&b, R_NOM);
    b.inv_v = 0.020;
    b.inv_hz = 14172.0;
    board_start(&b);
    board_run(&b, 120, r, NULL, 400000U);
    double far_ue = fabs(strain_ue(std_out(r, 120)));

    // 225 Hz: tick 14.4 kHz, the same tone aliases to 228 Hz
    board_defaults(&b, R_NOM);
    b.inv_v = 0.020;
    b.inv_hz = 14172.0;
    board_start(&b);
    TEST_ASSERT_TRUE(exc_runner_api_request(&runner, 225U, 0U, 0U));
    struct LockinResult res[150];
    int got = board_run(&b, 150, NULL, res, 800000U);
    int n = 0;
    for (int k = 0; k < got; k++) {
        if (res[k].exc_id == 1U) {
            bridge_api_convert(&BRIDGE, &res[k], &r[n++]);
        }
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT(100, n);
    double near_ue = fabs(strain_ue(std_out(r, n)));

    printf("INFO inverter 14172 Hz, 20 mV: rumore %.2f ue a 125 Hz (base %.2f), %.1f ue a 225 Hz\n", far_ue,
           fabs(floor_ue), near_ue);
    TEST_ASSERT_TRUE(far_ue < fabs(floor_ue) + 0.3);
    TEST_ASSERT_TRUE(near_ue > 20.0 * fabs(floor_ue));
}

// The DAC sags under the bridge: the zero does not move, the scale does (by the sag).
// A force calibration on the board absorbs it; CH1 on the bridge supply would remove it.
void test_chain_dac_sag_scales_only(void) {
    struct BridgeReading r[4];
    struct Board b;
    board_defaults(&b, R_NOM);
    b.dac_gain = 0.8;
    board_start(&b);
    board_run(&b, 4, r, NULL, 200000U);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, r[3].out_mv_per_v);

    board_defaults(&b, 345.0);
    b.dac_gain = 0.8;
    board_start(&b);
    board_run(&b, 4, r, NULL, 200000U);
    double expected = 0.8 * 1000.0 * imbalance(345.0);
    TEST_ASSERT_FLOAT_WITHIN((float)(0.003 * fabs(expected)), (float)expected, r[3].out_mv_per_v);
}

// A frequency change while measuring, as from the CAN command: applied only at the end of a
// period, with the DC level of the new table, never mixed into a result, first window dropped;
// a second request waits; impossible ones change nothing; an amplitude change keeps mV/V.
void test_chain_switch_sequence(void) {
    struct Board b;
    struct LockinResult res;
    struct ExcStep step;
    struct BridgeReading reading;
    double expected = 1000.0 * imbalance(345.0);
    board_defaults(&b, 345.0);
    board_start(&b);

    int results = 0;
    while (results < 3) {
        results += board_tick(&b, &res, NULL) ? 1 : 0;
    }
    for (int k = 0; k < 37; k++) {
        board_tick(&b, &res, NULL); // somewhere inside a period
    }
    TEST_ASSERT_FALSE(exc_runner_api_request(&runner, 500U, 0U, 0U));  // mains harmonic
    TEST_ASSERT_FALSE(exc_runner_api_request(&runner, 0U, 0U, 3000U)); // beyond the DAC range
    TEST_ASSERT_EQUAL_UINT8(0, runner.active->id);
    TEST_ASSERT_TRUE(exc_runner_api_request(&runner, 333U, 0U, 0U));
    TEST_ASSERT_FALSE(exc_runner_api_request(&runner, 175U, 0U, 0U)); // the first is pending

    uint32_t old_samples = runner.active->plan.samples;
    uint32_t ticks = 0;
    do {
        board_tick(&b, &res, &step);
        ticks++;
    } while (!step.switched && ticks < 100000U);
    TEST_ASSERT_TRUE(step.switched);
    TEST_ASSERT_EQUAL_UINT32(old_samples - 1U, step.phase); // end of the old period
    const struct ExcSetting *exc = runner.active;
    TEST_ASSERT_EQUAL_UINT8(1, exc->id);
    TEST_ASSERT_EQUAL_UINT16(exc->dac[0], step.dac);
    TEST_ASSERT_EQUAL_UINT32(exc->plan.autoreload, step.autoreload);
    TEST_ASSERT_INT_WITHIN(1, 683, step.dac); // 0.55 V: the DC level, no jump

    uint32_t window = exc->plan.samples * exc->plan.periods;
    uint32_t since = 0;
    int checked = 0;
    while (checked < 20) {
        since++;
        if (!board_tick(&b, &res, NULL)) {
            continue;
        }
        TEST_ASSERT_EQUAL_UINT8(1, res.exc_id);
        // the first result is dropped: no reported window holds the first period after the
        // switch (a tick at the old period and the settling of the amplifier)
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(window + exc->plan.samples, since);
        bridge_api_convert(&BRIDGE, &res, &reading);
        TEST_ASSERT_FLOAT_WITHIN((float)(0.03 * fabs(expected)), (float)expected, reading.out_mv_per_v);
        checked++;
    }
    double at_300mv = reading.out_mv_per_v;

    // bigger excitation, same frequency: same mV/V. 500 mV on the 550 mV DC would go below
    // the DAC range: the DC has to rise with it
    TEST_ASSERT_FALSE(exc_runner_api_request(&runner, 0U, 0U, 500U));
    TEST_ASSERT_TRUE(exc_runner_api_request(&runner, 0U, 700U, 500U));
    checked = 0;
    while (checked < 20) {
        if (board_tick(&b, &res, NULL) && res.exc_id == 2U) {
            checked++;
        }
    }
    TEST_ASSERT_EQUAL_UINT16(333, runner.active->freq_req_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.5F, res.amp_v);
    bridge_api_convert(&BRIDGE, &res, &reading);
    TEST_ASSERT_FLOAT_WITHIN((float)(0.003 * fabs(at_300mv)), (float)at_300mv, reading.out_mv_per_v);
}

// A lost ADC sample restarts the window: the next result is flagged and still right,
// the one after is clean.
void test_chain_spi_glitch(void) {
    struct Board b;
    struct LockinResult res[3];
    struct BridgeReading r;
    double expected = 1000.0 * imbalance(345.0);
    board_defaults(&b, 345.0);
    board_start(&b);
    board_run(&b, 3, NULL, res, 200000U);
    for (int k = 0; k < 100; k++) {
        board_tick(&b, &res[0], NULL);
    }
    b.spi_fail_next = true;
    board_run(&b, 2, NULL, res, 200000U);
    TEST_ASSERT_TRUE(res[0].spi_error);
    bridge_api_convert(&BRIDGE, &res[0], &r);
    TEST_ASSERT_BITS_HIGH(BRIDGE_FLAG_SPI_ERROR, r.flags);
    TEST_ASSERT_FLOAT_WITHIN((float)(0.003 * fabs(expected)), (float)expected, r.out_mv_per_v);
    TEST_ASSERT_FALSE(res[1].spi_error);
}

// The first ADC sample after start-up is not the selected channel: never reported.
void test_chain_startup_garbage_dropped(void) {
    struct Board b;
    struct LockinResult res[1];
    board_defaults(&b, R_NOM);
    b.garbage_next = true;
    board_start(&b);
    board_run(&b, 1, NULL, res, 200000U);
    TEST_ASSERT_TRUE(res[0].max < 65535U);
}

// A tick that outlasted the timer period marks the window in progress, only that one.
void test_chain_overrun_flag(void) {
    struct Board b;
    struct LockinResult res[2];
    struct BridgeReading r;
    board_defaults(&b, R_NOM);
    board_start(&b);
    board_run(&b, 2, NULL, res, 200000U);
    exc_runner_api_mark_overrun(&runner);
    board_run(&b, 2, NULL, res, 200000U);
    bridge_api_convert(&BRIDGE, &res[0], &r);
    TEST_ASSERT_BITS_HIGH(BRIDGE_FLAG_OVERRUN, r.flags);
    TEST_ASSERT_FALSE(res[1].overrun);
}

// CAN frames byte for byte as acquisinator.dbc decodes them (bytes from cantools).
void test_chain_can_frames_match_dbc(void) {
    uint8_t data[8];
    struct LockinResult res = { 0 };
    struct BridgeReading reading = { 0 };

    res.dc = 12345.0F;
    res.spi_error = true;
    res.exc_id = 7;
    reading.x_v = -0.0123F;
    bridge_api_pack_diag(&res, &reading, data);
    static const uint8_t DIAG[8] = { 0xF0, 0x85, 0x49, 0xBC, 0x39, 0x30, 0x01, 0x07 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DIAG, data, 8);

    reading.out_mv_per_v = 0.0456F;
    reading.phase_deg = -12.34F;
    reading.flags = 0x25;
    bridge_api_pack_bridge(&reading, 200, data);
    static const uint8_t BRIDGE_FRAME[8] = { 0x11, 0xC7, 0x3A, 0x3D, 0x2E, 0xFB, 0x25, 0xC8 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(BRIDGE_FRAME, data, 8);

    bridge_api_pack_no_data(true, data);
    static const uint8_t NO_DATA[8] = { 0x00, 0x00, 0xC0, 0x7F, 0x00, 0x00, 0x14, 0x00 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NO_DATA, data, 8);

    struct ExcSetting exc = { 0 };
    exc.plan.freq_hz = 174.9F;
    exc.plan.samples = 64;
    exc.plan.periods = 14;
    exc.amp_mv = 1450;
    exc.dc_mv = 1650;
    exc.id = 3;
    exc_runner_api_pack_status(&exc, data);
    static const uint8_t EXC[8] = { 0xD5, 0x06, 0x40, 0x0E, 0xAA, 0x25, 0x67, 0x03 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(EXC, data, 8);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_chain_electronics_expected_values);
    RUN_TEST(test_chain_large_imbalance_and_clipping);
    RUN_TEST(test_chain_all_frequencies_ideal_analog);
    RUN_TEST(test_chain_all_frequencies_amplifier_pole);
    RUN_TEST(test_chain_mains_at_default_frequency);
    RUN_TEST(test_chain_inverter_alias);
    RUN_TEST(test_chain_dac_sag_scales_only);
    RUN_TEST(test_chain_switch_sequence);
    RUN_TEST(test_chain_spi_glitch);
    RUN_TEST(test_chain_startup_garbage_dropped);
    RUN_TEST(test_chain_overrun_flag);
    RUN_TEST(test_chain_can_frames_match_dbc);
    return UNITY_END();
}
