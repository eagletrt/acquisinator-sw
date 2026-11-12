#include <unity.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "exc-plan-api.h"
#include "sync-demod-api.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ADC_MAX     65535U
#define ADC_MID     32767.5
#define F_MAX_TEST  2000U  // sweep beyond the 1 kHz limit to see the rejection too

// Same configuration as the firmware (main.c)
static const struct ExcPlanConfig CFG = {
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
};

static int16_t sin_tab[64];
static int16_t cos_tab[64];

void setUp(void) {}
void tearDown(void) {}

// The default setting keeps exactly the timing it had with fixed constants.
void test_exc_plan_default_unchanged(void) {
    struct ExcPlan plan;
    TEST_ASSERT_TRUE(exc_plan_api_make(&CFG, 125U, &plan));
    TEST_ASSERT_EQUAL_UINT32(64U, plan.samples);
    TEST_ASSERT_EQUAL_UINT32(8999U, plan.autoreload);
    TEST_ASSERT_EQUAL_UINT32(5U, plan.periods);
    TEST_ASSERT_EQUAL_UINT32(1U, plan.output_every);
    TEST_ASSERT_EQUAL_FLOAT(125.0F, plan.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.040F, plan.window_s);
}

// Every accepted plan, for every frequency the CAN command can ask in the range, keeps
// the invariants the ISR and the demodulator rely on.
void test_exc_plan_invariants(void) {
    uint32_t accepted = 0;
    for (uint32_t f = 0; f <= F_MAX_TEST; f++) {
        struct ExcPlan plan;
        if (!exc_plan_api_make(&CFG, f, &plan)) {
            continue;
        }
        accepted++;
        uint64_t reload = (uint64_t)plan.autoreload + 1U;
        double tick_hz = (double)CFG.tim_clk_hz / (double)reload;
        double freq = tick_hz / plan.samples;
        double window = (double)plan.periods / freq;

        // ISR budget and table length
        TEST_ASSERT_TRUE(tick_hz <= CFG.max_tick_hz);
        TEST_ASSERT_TRUE(plan.samples >= CFG.min_samples && plan.samples <= CFG.max_samples);
        // what is generated is what was asked, up to the timer rounding
        TEST_ASSERT_TRUE(fabs(freq - f) / f < 1e-3);
        TEST_ASSERT_FLOAT_WITHIN(1e-3F * (float)freq, (float)freq, plan.freq_hz);
        // window: whole periods inside the ring, within the limits, whole mains periods
        TEST_ASSERT_TRUE(plan.periods >= 1U && plan.periods <= CFG.max_periods);
        TEST_ASSERT_TRUE(window * 1000.0 >= CFG.min_window_ms - 1e-6);
        TEST_ASSERT_TRUE(window * 1000.0 <= CFG.max_window_ms + 1e-6);
        double mains_periods = round(50.0 * window);
        TEST_ASSERT_TRUE(mains_periods >= 1.0);
        TEST_ASSERT_TRUE(fabs(mains_periods / window - 50.0) <= CFG.mains_tol_hz + 1e-4);
        // never next to a mains harmonic
        double harmonic = round(freq / 50.0);
        TEST_ASSERT_TRUE(harmonic < 1.0 || fabs(freq - harmonic * 50.0) >= CFG.mains_guard_hz - 1e-3);
        // results about every output_ms, never more than one per period
        TEST_ASSERT_TRUE(plan.output_every >= 1U);
        TEST_ASSERT_TRUE(plan.output_every == 1U || fabs(plan.output_every - freq * CFG.output_ms / 1000.0) <= 0.5 + 1e-3);
    }
    // nothing above the tick budget with the fewest samples
    struct ExcPlan plan;
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, CFG.max_tick_hz / CFG.min_samples + 1U, &plan));
    TEST_ASSERT_GREATER_THAN_UINT32(200U, accepted);
}

// Values the command cannot sensibly carry are rejected without wrapping.
void test_exc_plan_rejects(void) {
    struct ExcPlan plan;
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, 0U, &plan));
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, 500U, &plan)); // mains harmonic
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, 100U, &plan));
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, 153U, &plan)); // 3 Hz from 150
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, 47U, &plan));
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, UINT16_MAX, &plan));
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, UINT32_MAX, &plan));
    TEST_ASSERT_FALSE(exc_plan_api_make(&CFG, (1UL << 28) + 125U, &plan)); // * 16 wraps
}

// The frequencies of the old fixed list are all still there.
void test_exc_plan_old_list_accepted(void) {
    static const uint32_t OLD[] = { 25U, 75U, 125U, 225U };
    for (uint32_t n = 0; n < sizeof(OLD) / sizeof(OLD[0]); n++) {
        struct ExcPlan plan;
        TEST_ASSERT_TRUE(exc_plan_api_make(&CFG, OLD[n], &plan));
    }
}

// Run one whole window of dc + amp*sin + mains at `mains_hz` (fundamental + 3rd harmonic,
// as the bridge picks it up) through the demodulator with the plan's timing.
static void run_plan(const struct ExcPlan *plan, double amp, double mains, double mains_hz, double phase,
                     struct SyncDemodHandler *h) {
    sync_demod_api_make_tables(sin_tab, cos_tab, plan->samples);
    sync_demod_api_init(h, sin_tab, cos_tab, plan->samples, plan->periods);
    double tick_hz = 72000000.0 / ((double)plan->autoreload + 1.0);
    uint32_t n = plan->samples * plan->periods;
    int results = 0;
    for (uint32_t k = 0; k < n; k++) {
        uint32_t idx = k % plan->samples;
        double t = (double)k / tick_hz;
        double x = ADC_MID + amp * sin(2.0 * M_PI * idx / plan->samples) +
                   mains * (sin(2.0 * M_PI * mains_hz * t + phase) + 0.3 * sin(2.0 * M_PI * 3.0 * mains_hz * t + phase));
        results += sync_demod_api_update(h, (uint16_t)lround(x), idx) ? 1 : 0;
    }
    TEST_ASSERT_EQUAL_INT(1, results);
}

// Expected leak of the test mains (fundamental + 0.3 * 3rd harmonic) when it is `d` Hz off
// the whole-period frequency of the window: k * d / distance, for f - k*50 and f + k*50.
static double mains_leak_bound(double f, double d) {
    static const double AMPL[] = { 1.0, 0.0, 0.3 };
    double bound = 0.0;
    for (uint32_t k = 1; k <= 3U; k++) {
        double h = 50.0 * k;
        bound += AMPL[k - 1U] * k * d * (1.0 / fabs(f - h) + 1.0 / (f + h));
    }
    return bound;
}

// For every accepted frequency the mains leaks no more than the model in exc-plan.h says,
// with the grid exactly at 50 Hz and off by +-0.1 Hz: the leak is predictable from the plan.
void test_exc_plan_rejects_mains_everywhere(void) {
    const double amp = 5000.0;
    const double mains = 15000.0;
    static const double MAINS_HZ[] = { 49.9, 50.0, 50.1 };
    static const double PHASES[] = { 0.0, 1.1, 2.3 };
    for (uint32_t f = 1; f <= F_MAX_TEST; f++) {
        struct ExcPlan plan;
        if (!exc_plan_api_make(&CFG, f, &plan)) {
            continue;
        }
        for (uint32_t m = 0; m < 3U; m++) {
            double d = fabs(MAINS_HZ[m] - 50.0) + plan.mains_err_hz;
            double allowed = 1.3 * mains * mains_leak_bound(plan.freq_hz, d) + 3.0;
            for (uint32_t p = 0; p < 3U; p++) {
                struct SyncDemodHandler h;
                run_plan(&plan, amp, mains, MAINS_HZ[m], PHASES[p], &h);
                double leak = hypot(h.i - amp, h.q);
                if (leak > allowed) {
                    char msg[112];
                    snprintf(msg, sizeof(msg), "f=%u Hz, rete %.1f Hz: errore %.1f, atteso <= %.1f", (unsigned)f,
                             MAINS_HZ[m], leak, allowed);
                    TEST_FAIL_MESSAGE(msg);
                }
            }
        }
    }
}

// At the default 125 Hz the grid drift leaks as little as before (under 0.3% of the mains).
void test_exc_plan_default_mains_leak(void) {
    struct ExcPlan plan;
    TEST_ASSERT_TRUE(exc_plan_api_make(&CFG, 125U, &plan));
    for (double mains_hz = 49.9; mains_hz < 50.15; mains_hz += 0.1) {
        struct SyncDemodHandler h;
        run_plan(&plan, 5000.0, 15000.0, mains_hz, 1.1, &h);
        TEST_ASSERT_TRUE(hypot(h.i - 5000.0, h.q) < 0.003 * 15000.0);
    }
}

// Full-scale square wave (0 / 65535) in phase with the reference: the largest products the
// accumulators can take. For every accepted plan the result is the sampled fundamental.
void test_exc_plan_full_scale_no_overflow(void) {
    for (uint32_t f = 1; f <= F_MAX_TEST; f++) {
        struct ExcPlan plan;
        if (!exc_plan_api_make(&CFG, f, &plan)) {
            continue;
        }
        uint32_t n_s = plan.samples;
        sync_demod_api_make_tables(sin_tab, cos_tab, n_s);
        double expected = 0.0;
        for (uint32_t k = 0; k < n_s; k++) {
            double s = sin(2.0 * M_PI * k / n_s);
            double x = s > 1e-9 ? ADC_MAX : (s < -1e-9 ? 0.0 : (double)(ADC_MAX / 2U));
            expected += 2.0 / n_s * x * s;
        }
        struct SyncDemodHandler h;
        sync_demod_api_init(&h, sin_tab, cos_tab, n_s, plan.periods);
        int results = 0;
        for (uint32_t k = 0; k < n_s * plan.periods; k++) {
            uint32_t idx = k % n_s;
            double s = sin(2.0 * M_PI * idx / n_s);
            uint16_t x = s > 1e-9 ? (uint16_t)ADC_MAX : (s < -1e-9 ? 0U : (uint16_t)(ADC_MAX / 2U));
            results += sync_demod_api_update(&h, x, idx) ? 1 : 0;
        }
        TEST_ASSERT_EQUAL_INT(1, results);
        TEST_ASSERT_FLOAT_WITHIN((float)(2e-3 * expected), (float)expected, h.i);
        TEST_ASSERT_EQUAL_UINT16(0, h.min);
        TEST_ASSERT_EQUAL_UINT16(ADC_MAX, h.max);
    }
}

// Same worst case over the longest window the ring holds.
void test_exc_plan_full_scale_longest_window(void) {
    sync_demod_api_make_tables(sin_tab, cos_tab, 64U);
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, 64U, SYNC_DEMOD_MAX_PERIODS);
    int results = 0;
    for (uint32_t k = 0; k < 64U * SYNC_DEMOD_MAX_PERIODS; k++) {
        uint32_t idx = k % 64U;
        uint16_t x = (idx > 0U && idx < 32U) ? (uint16_t)ADC_MAX : (idx == 0U || idx == 32U ? (uint16_t)(ADC_MAX / 2U) : 0U);
        results += sync_demod_api_update(&h, x, idx) ? 1 : 0;
    }
    TEST_ASSERT_EQUAL_INT(1, results);
    TEST_ASSERT_FLOAT_WITHIN(0.002F * 41722.0F, 41722.0F, h.i);
    TEST_ASSERT_FLOAT_WITHIN(1.0F, (float)ADC_MID, h.dc);
}

// DAC levels: only inside the buffered output range (0.2 .. VDDA - 0.2 V, as in main.c),
// and the table spans exactly dc +- amp.
void test_exc_plan_levels(void) {
    const float lo = 0.2F;
    const float hi = 3.1F;
    TEST_ASSERT_TRUE(exc_plan_api_levels_valid(0.55F, 0.30F, lo, hi)); // default
    TEST_ASSERT_TRUE(exc_plan_api_levels_valid(1.65F, 1.45F, lo, hi)); // widest
    TEST_ASSERT_FALSE(exc_plan_api_levels_valid(1.65F, 1.46F, lo, hi));
    TEST_ASSERT_FALSE(exc_plan_api_levels_valid(0.55F, 0.0F, lo, hi));
    TEST_ASSERT_FALSE(exc_plan_api_levels_valid(0.45F, 0.30F, lo, hi)); // down to 0.15 V
    TEST_ASSERT_FALSE(exc_plan_api_levels_valid(2.9F, 0.25F, lo, hi));  // up to 3.15 V
    TEST_ASSERT_FALSE(exc_plan_api_levels_valid(65.535F, 65.535F, lo, hi)); // u16 mV from CAN

    static const uint32_t N[] = { 16U, 21U, 64U };
    for (uint32_t n = 0; n < 3U; n++) {
        uint16_t dac[64];
        sync_demod_api_make_tables(sin_tab, cos_tab, N[n]);
        exc_plan_api_make_dac_table(dac, sin_tab, N[n], 1.65F, 1.65F, 3.3F, 4095U);
        uint16_t lo = UINT16_MAX;
        uint16_t hi = 0;
        for (uint32_t k = 0; k < N[n]; k++) {
            lo = dac[k] < lo ? dac[k] : lo;
            hi = dac[k] > hi ? dac[k] : hi;
        }
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(4095U, hi);
        TEST_ASSERT_EQUAL_UINT16(2048U, dac[0]); // sin(0): the DC level
        if (N[n] % 4U == 0U) {
            TEST_ASSERT_EQUAL_UINT16(4095U, hi);
            TEST_ASSERT_EQUAL_UINT16(0U, lo);
        }
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_exc_plan_default_unchanged);
    RUN_TEST(test_exc_plan_invariants);
    RUN_TEST(test_exc_plan_rejects);
    RUN_TEST(test_exc_plan_old_list_accepted);
    RUN_TEST(test_exc_plan_rejects_mains_everywhere);
    RUN_TEST(test_exc_plan_default_mains_leak);
    RUN_TEST(test_exc_plan_full_scale_no_overflow);
    RUN_TEST(test_exc_plan_full_scale_longest_window);
    RUN_TEST(test_exc_plan_levels);
    return UNITY_END();
}
