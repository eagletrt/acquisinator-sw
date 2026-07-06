#include <unity.h>
#include <math.h>
#include "lockin-api.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS      1000.0f
#define FREF    50.0f
#define FCUT    5.0f
#define REF_AMP 1.0f

void setUp(void) {}
void tearDown(void) {}

// Excite the lock-in with reference = REF_AMP*cos(wt) and an in-phase
// signal = sig_gain*reference, returning the last output after `n` samples.
static float run_lockin(struct LockInHandler *h, float sig_gain, int n) {
    lockin_api_init(h, FREF, FCUT, FS, REF_AMP);
    float w = 2.0f * (float)M_PI * FREF / FS;
    float out = 0.0f;
    for (int k = 0; k < n; k++) {
        float ref = REF_AMP * cosf(w * (float)k);
        float sig = sig_gain * ref;
        out = lockin_api_update(h, sig, ref);
    }
    return out;
}

void test_lockin_init_sets_reference_and_zeroes(void) {
    struct LockInHandler h;
    lockin_api_init(&h, FREF, FCUT, FS, REF_AMP);
    TEST_ASSERT_EQUAL_FLOAT(REF_AMP, h.ref_amplitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.amplitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.phase);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.output);
}

// The returned value must be the freshly computed output field.
void test_lockin_update_returns_output_field(void) {
    struct LockInHandler h;
    float ret = run_lockin(&h, 0.5f, 2000);
    TEST_ASSERT_EQUAL_FLOAT(h.output, ret);
}

// output == amplitude / (ref_amplitude^2 + LOCKIN_OFFSET), by construction.
void test_lockin_output_matches_transfer_relation(void) {
    struct LockInHandler h;
    run_lockin(&h, 0.5f, 2000);
    float expected = h.amplitude / (h.ref_amplitude * h.ref_amplitude + LOCKIN_OFFSET);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected, h.output);
}

// No excitation settles to zero output.
void test_lockin_zero_signal_gives_zero_output(void) {
    struct LockInHandler h;
    float out = run_lockin(&h, 0.0f, 2000);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
}

// A coherent excitation is detected: the output is clearly non-zero.
void test_lockin_detects_coherent_signal(void) {
    struct LockInHandler h;
    float out = run_lockin(&h, 0.5f, 4000);
    TEST_ASSERT_TRUE_MESSAGE(out > 0.05f, "coherent signal must produce a sizeable output");
}

// The chain is linear: doubling the signal gain doubles the output.
void test_lockin_output_scales_linearly(void) {
    struct LockInHandler h1, h2;
    float o1 = run_lockin(&h1, 0.5f, 4000);
    float o2 = run_lockin(&h2, 1.0f, 4000);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, o2 / o1);
}

void test_lockin_reset_clears_state(void) {
    struct LockInHandler h;
    run_lockin(&h, 0.5f, 500);
    lockin_api_reset(&h);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.amplitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.phase);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, h.output);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_lockin_init_sets_reference_and_zeroes);
    RUN_TEST(test_lockin_update_returns_output_field);
    RUN_TEST(test_lockin_output_matches_transfer_relation);
    RUN_TEST(test_lockin_zero_signal_gives_zero_output);
    RUN_TEST(test_lockin_detects_coherent_signal);
    RUN_TEST(test_lockin_output_scales_linearly);
    RUN_TEST(test_lockin_reset_clears_state);
    return UNITY_END();
}
