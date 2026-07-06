#include <unity.h>
#include "sine-lut-api.h"

#define SIZE 256
#define FULL_SCALE 4095

static uint16_t buffer[SIZE];
static struct SineLUT lut;

void setUp(void) {
    sine_lut_api_init(&lut, buffer, SIZE, FULL_SCALE);
}

void tearDown(void) {
}

void test_sine_lut_init_sets_geometry(void) {
    TEST_ASSERT_EQUAL_PTR(buffer, lut.samples);
    TEST_ASSERT_EQUAL_UINT32(SIZE, lut.size);
    TEST_ASSERT_EQUAL_UINT32(0, lut.index);
}

// Characteristic points of one period: mid-scale at 0, peak at a quarter,
// mid-scale at half, trough at three quarters.
void test_sine_lut_characteristic_points(void) {
    TEST_ASSERT_UINT16_WITHIN(1, FULL_SCALE / 2, buffer[0]);
    TEST_ASSERT_EQUAL_UINT16(FULL_SCALE, buffer[SIZE / 4]);
    TEST_ASSERT_UINT16_WITHIN(1, FULL_SCALE / 2, buffer[SIZE / 2]);
    TEST_ASSERT_EQUAL_UINT16(0, buffer[3 * SIZE / 4]);
}

// Every sample must stay within the DAC range [0, FULL_SCALE].
void test_sine_lut_samples_within_range(void) {
    for (int i = 0; i < SIZE; i++) {
        TEST_ASSERT_TRUE_MESSAGE(buffer[i] <= FULL_SCALE, "sample exceeds full scale");
    }
}

// One period is symmetric: sample[i] and sample[size/2 + i] are mirrored
// around the mid-scale value.
void test_sine_lut_is_half_period_symmetric(void) {
    for (int i = 1; i < SIZE / 2; i++) {
        int sum = (int)buffer[i] + (int)buffer[SIZE / 2 + i];
        TEST_ASSERT_INT_WITHIN(2, FULL_SCALE, sum);
    }
}

// next() walks the table and wraps around after `size` reads.
void test_sine_lut_next_advances_and_wraps(void) {
    TEST_ASSERT_EQUAL_UINT16(buffer[0], sine_lut_api_next(&lut));
    TEST_ASSERT_EQUAL_UINT16(buffer[1], sine_lut_api_next(&lut));
    for (int i = 2; i < SIZE; i++) {
        sine_lut_api_next(&lut);
    }
    // After SIZE reads the index has wrapped back to the start.
    TEST_ASSERT_EQUAL_UINT32(0, lut.index);
    TEST_ASSERT_EQUAL_UINT16(buffer[0], sine_lut_api_next(&lut));
}

void test_sine_lut_reset_rewinds_index(void) {
    sine_lut_api_next(&lut);
    sine_lut_api_next(&lut);
    sine_lut_api_reset(&lut);
    TEST_ASSERT_EQUAL_UINT32(0, lut.index);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_sine_lut_init_sets_geometry);
    RUN_TEST(test_sine_lut_characteristic_points);
    RUN_TEST(test_sine_lut_samples_within_range);
    RUN_TEST(test_sine_lut_is_half_period_symmetric);
    RUN_TEST(test_sine_lut_next_advances_and_wraps);
    RUN_TEST(test_sine_lut_reset_rewinds_index);
    return UNITY_END();
}
