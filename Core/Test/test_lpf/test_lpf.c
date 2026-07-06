#include <unity.h>
#include "lpf-api.h"

#define FS 1000.0f
#define FC 10.0f

void setUp(void) {}
void tearDown(void) {}

// After init the coefficient must be a valid one-pole alpha and state cleared.
void test_lpf_init_sets_valid_alpha(void) {
    struct LPFHandler lpf;
    lpf_api_init(&lpf, FC, FS);
    TEST_ASSERT_TRUE_MESSAGE(lpf.alpha > 0.0f && lpf.alpha < 1.0f, "alpha must lie in (0, 1)");
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lpf.prev_output);
}

// With zero initial state the first output is exactly alpha * input.
void test_lpf_first_step_equals_alpha_times_input(void) {
    struct LPFHandler lpf;
    lpf_api_init(&lpf, FC, FS);
    float out = lpf_api_update(&lpf, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, lpf.alpha, out);
}

// A constant input must converge to that constant (unity DC gain).
void test_lpf_converges_to_constant_input(void) {
    struct LPFHandler lpf;
    lpf_api_init(&lpf, FC, FS);
    float out = 0.0f;
    for (int i = 0; i < 2000; i++) {
        out = lpf_api_update(&lpf, 3.3f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.3f, out);
}

void test_lpf_reset_clears_state(void) {
    struct LPFHandler lpf;
    lpf_api_init(&lpf, FC, FS);
    lpf_api_update(&lpf, 5.0f);
    lpf_api_reset(&lpf);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lpf.prev_output);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_lpf_init_sets_valid_alpha);
    RUN_TEST(test_lpf_first_step_equals_alpha_times_input);
    RUN_TEST(test_lpf_converges_to_constant_input);
    RUN_TEST(test_lpf_reset_clears_state);
    return UNITY_END();
}
