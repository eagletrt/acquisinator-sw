#include <unity.h>
#include <math.h>
#include <stdint.h>
#include "sync-demod-api.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PERIOD  64U
#define PERIODS 25U
#define DC      20000.0f

static int16_t sin_tab[PERIOD];
static int16_t cos_tab[PERIOD];

void setUp(void) {
    sync_demod_api_make_tables(sin_tab, cos_tab, PERIOD);
}
void tearDown(void) {}

// Feed one full window of dc + amp*sin(2*pi*k/PERIOD + phi) and return
// whether the last sample (and only the last) completed the window.
static int run_window(struct SyncDemodHandler *h, float amp, float phi) {
    int completed = 0;
    for (uint32_t k = 0; k < PERIOD * PERIODS; k++) {
        uint32_t idx = k % PERIOD;
        float theta = 2.0f * (float)M_PI * (float)idx / (float)PERIOD;
        uint16_t x = (uint16_t)lroundf(DC + amp * sinf(theta + phi));
        if (sync_demod_api_update(h, x, idx)) {
            completed++;
            TEST_ASSERT_EQUAL_UINT32(PERIOD * PERIODS - 1U, k);
        }
    }
    return completed;
}

void test_sync_demod_tables(void) {
    TEST_ASSERT_EQUAL_INT16(0, sin_tab[0]);
    TEST_ASSERT_EQUAL_INT16(SYNC_DEMOD_Q15_ONE, cos_tab[0]);
    TEST_ASSERT_EQUAL_INT16(SYNC_DEMOD_Q15_ONE, sin_tab[PERIOD / 4]);
    TEST_ASSERT_EQUAL_INT16(-SYNC_DEMOD_Q15_ONE, cos_tab[PERIOD / 2]);
}

void test_sync_demod_window_completes_once(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    TEST_ASSERT_EQUAL_INT(1, run_window(&h, 1000.0f, 0.0f));
    TEST_ASSERT_EQUAL_INT(1, run_window(&h, 1000.0f, 0.0f));
}

// Pure DC must give no AC component, only the mean.
void test_sync_demod_rejects_dc(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    run_window(&h, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, h.i);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, h.q);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, DC, h.dc);
}

// In-phase signal: amplitude in i, nothing in q.
void test_sync_demod_in_phase(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    run_window(&h, 1000.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, h.i);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, h.q);
}

// Opposite phase keeps its sign: this is what tells R above from R below balance.
void test_sync_demod_antiphase_is_negative(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    run_window(&h, 1000.0f, (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -1000.0f, h.i);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, h.q);
}

// A 90 degree shift moves the amplitude to q; a generic shift splits it.
void test_sync_demod_phase_shift(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    run_window(&h, 1000.0f, (float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, h.i);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, h.q);

    float phi = 0.5f;
    run_window(&h, 1000.0f, phi);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f * cosf(phi), h.i);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f * sinf(phi), h.q);
}

void test_sync_demod_min_max(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    run_window(&h, 1000.0f, 0.0f);
    TEST_ASSERT_UINT16_WITHIN(1, (uint16_t)(DC - 1000.0f), h.min);
    TEST_ASSERT_UINT16_WITHIN(1, (uint16_t)(DC + 1000.0f), h.max);
}

// Reset drops a partial window: the next full window is not polluted by it.
void test_sync_demod_reset_drops_partial_window(void) {
    struct SyncDemodHandler h;
    sync_demod_api_init(&h, sin_tab, cos_tab, PERIOD, PERIODS);
    for (uint32_t k = 0; k < PERIOD * 3U; k++) {
        sync_demod_api_update(&h, 60000, k % PERIOD);
    }
    sync_demod_api_reset(&h);
    TEST_ASSERT_EQUAL_INT(1, run_window(&h, 1000.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, h.i);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, DC, h.dc);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_sync_demod_tables);
    RUN_TEST(test_sync_demod_window_completes_once);
    RUN_TEST(test_sync_demod_rejects_dc);
    RUN_TEST(test_sync_demod_in_phase);
    RUN_TEST(test_sync_demod_antiphase_is_negative);
    RUN_TEST(test_sync_demod_phase_shift);
    RUN_TEST(test_sync_demod_min_max);
    RUN_TEST(test_sync_demod_reset_drops_partial_window);
    return UNITY_END();
}
