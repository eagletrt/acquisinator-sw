#include <unity.h>
#include <math.h>
#include "demod-api.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS   1000.0f
#define FREF 50.0f
#define FCUT 5.0f

void setUp(void) {}
void tearDown(void) {}

// Drive the demodulator with reference = ref_amp*cos(w_ref*k) and
// signal = sig_amp*cos(w_sig*k), then return the recovered amplitude
// after `n` samples. Pass sig_freq == FREF for a coherent excitation.
static float run_demod(struct DemodHandler *d, float ref_amp, float sig_amp, float sig_freq, int n) {
    demod_api_init(d, FREF, FCUT, FS);
    float w_ref = 2.0f * (float)M_PI * FREF / FS;
    float w_sig = 2.0f * (float)M_PI * sig_freq / FS;
    for (int k = 0; k < n; k++) {
        float ref = ref_amp * cosf(w_ref * (float)k);
        float sig = sig_amp * cosf(w_sig * (float)k);
        demod_api_update(d, sig, ref);
    }
    return d->amplitude;
}

void test_demod_init_zeroes_state(void) {
    struct DemodHandler d;
    demod_api_init(&d, FREF, FCUT, FS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.i);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.q);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.amplitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.phase);
}

// No excitation must settle to zero amplitude.
void test_demod_zero_signal_gives_zero_amplitude(void) {
    struct DemodHandler d;
    float amp = run_demod(&d, 1.0f, 0.0f, FREF, 2000);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, amp);
}

// A coherent signal is detected: the recovered amplitude is clearly non-zero.
void test_demod_detects_coherent_signal(void) {
    struct DemodHandler d;
    float amp = run_demod(&d, 1.0f, 0.5f, FREF, 4000);
    TEST_ASSERT_TRUE_MESSAGE(amp > 0.1f, "coherent signal must produce a sizeable amplitude");
}

// The chain is linear up to the magnitude: doubling the signal amplitude
// doubles the recovered amplitude.
void test_demod_amplitude_scales_linearly(void) {
    struct DemodHandler d1, d2;
    float a1 = run_demod(&d1, 1.0f, 0.5f, FREF, 4000);
    float a2 = run_demod(&d2, 1.0f, 1.0f, FREF, 4000);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, a2 / a1);
}

// Frequency selectivity: an off-frequency signal is strongly rejected
// compared to a coherent one of equal amplitude.
void test_demod_rejects_off_frequency_signal(void) {
    struct DemodHandler dc, doff;
    float coherent = run_demod(&dc, 1.0f, 0.5f, FREF, 4000);
    float off      = run_demod(&doff, 1.0f, 0.5f, 200.0f, 4000);
    TEST_ASSERT_TRUE_MESSAGE(off < 0.25f * coherent, "off-frequency signal must be rejected");
}

void test_demod_reset_clears_state(void) {
    struct DemodHandler d;
    run_demod(&d, 1.0f, 0.5f, FREF, 500);
    demod_api_reset(&d);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.amplitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.i);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.q);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_demod_init_zeroes_state);
    RUN_TEST(test_demod_zero_signal_gives_zero_amplitude);
    RUN_TEST(test_demod_detects_coherent_signal);
    RUN_TEST(test_demod_amplitude_scales_linearly);
    RUN_TEST(test_demod_rejects_off_frequency_signal);
    RUN_TEST(test_demod_reset_clears_state);
    return UNITY_END();
}
