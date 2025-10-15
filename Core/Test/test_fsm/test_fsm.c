#include <unity.h>
#include "acquisinator_fsm.h"

void init_to_idle(state_data_t *data);
void to_fatal(state_data_t *data);
void stay(state_data_t *data);
void idle_to_flash(state_data_t *data);
void idle_to_calibration(state_data_t *data);
void flash_to_idle(state_data_t *data);
void calibration_to_idle(state_data_t *data);

void setUp(void) {
}

void tearDown(void) {
}

void test_state_names_are_correct(void) {
    TEST_ASSERT_EQUAL_STRING("init", state_names[STATE_INIT]);
    TEST_ASSERT_EQUAL_STRING("idle", state_names[STATE_IDLE]);
    TEST_ASSERT_EQUAL_STRING("fatal", state_names[STATE_FATAL]);
    TEST_ASSERT_EQUAL_STRING("flash", state_names[STATE_FLASH]);
    TEST_ASSERT_EQUAL_STRING("calibration", state_names[STATE_CALIBRATION]);
}

void test_init_state_transitions_to_idle(void) {
    state_t next_state = run_state(STATE_INIT, NULL);
    TEST_ASSERT_EQUAL(STATE_IDLE, next_state);
}

void test_idle_state_stays_idle(void) {
    state_t next_state = run_state(STATE_IDLE, NULL);
    TEST_ASSERT_EQUAL(STATE_IDLE, next_state);
}

void test_fatal_state_stays_fatal(void) {
    state_t next_state = run_state(STATE_FATAL, NULL);
    TEST_ASSERT_EQUAL(STATE_FATAL, next_state);
}

void test_flash_state_stays_flash(void) {
    state_t next_state = run_state(STATE_FLASH, NULL);
    TEST_ASSERT_EQUAL(STATE_FLASH, next_state);
}

void test_calibration_state_stays_calibration(void) {
    state_t next_state = run_state(STATE_CALIBRATION, NULL);
    TEST_ASSERT_EQUAL(STATE_CALIBRATION, next_state);
}

void test_state_table_has_all_state_functions(void) {
    TEST_ASSERT_NOT_NULL(state_table[STATE_INIT]);
    TEST_ASSERT_NOT_NULL(state_table[STATE_IDLE]);
    TEST_ASSERT_NOT_NULL(state_table[STATE_FATAL]);
    TEST_ASSERT_NOT_NULL(state_table[STATE_FLASH]);
    TEST_ASSERT_NOT_NULL(state_table[STATE_CALIBRATION]);
}

void test_transition_from_init_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_INIT][STATE_IDLE]);
}

void test_transition_from_init_to_fatal_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_INIT][STATE_FATAL]);
}

void test_transition_from_idle_to_flash_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_IDLE][STATE_FLASH]);
}

void test_transition_from_idle_to_calibration_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_IDLE][STATE_CALIBRATION]);
}

void test_transition_from_flash_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_FLASH][STATE_IDLE]);
}

void test_transition_from_calibration_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL(transition_table[STATE_CALIBRATION][STATE_IDLE]);
}

void test_invalid_transitions_are_null(void) {
    TEST_ASSERT_NULL(transition_table[STATE_INIT][STATE_INIT]);
    TEST_ASSERT_NULL(transition_table[STATE_FATAL][STATE_IDLE]);
    TEST_ASSERT_NULL(transition_table[STATE_FLASH][STATE_CALIBRATION]);
}

void test_do_init_returns_valid_state(void) {
    state_t result = do_init(NULL);
    TEST_ASSERT_TRUE(result == STATE_IDLE || result == STATE_FATAL);
}

void test_do_idle_returns_valid_state(void) {
    state_t result = do_idle(NULL);
    TEST_ASSERT_TRUE(result == NO_CHANGE || result == STATE_IDLE ||
                     result == STATE_FATAL || result == STATE_FLASH ||
                     result == STATE_CALIBRATION);
}

void test_do_fatal_returns_valid_state(void) {
    state_t result = do_fatal(NULL);
    TEST_ASSERT_TRUE(result == NO_CHANGE || result == STATE_FATAL);
}

void test_do_flash_returns_valid_state(void) {
    state_t result = do_flash(NULL);
    TEST_ASSERT_TRUE(result == NO_CHANGE || result == STATE_IDLE ||
                     result == STATE_FATAL || result == STATE_FLASH);
}

void test_do_calibration_returns_valid_state(void) {
    state_t result = do_calibration(NULL);
    TEST_ASSERT_TRUE(result == NO_CHANGE || result == STATE_IDLE ||
                     result == STATE_FATAL || result == STATE_CALIBRATION);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_state_names_are_correct);
    RUN_TEST(test_init_state_transitions_to_idle);
    RUN_TEST(test_idle_state_stays_idle);
    RUN_TEST(test_fatal_state_stays_fatal);
    RUN_TEST(test_flash_state_stays_flash);
    RUN_TEST(test_calibration_state_stays_calibration);
    RUN_TEST(test_state_table_has_all_state_functions);
    RUN_TEST(test_transition_from_init_to_idle_exists);
    RUN_TEST(test_transition_from_init_to_fatal_exists);
    RUN_TEST(test_transition_from_idle_to_flash_exists);
    RUN_TEST(test_transition_from_idle_to_calibration_exists);
    RUN_TEST(test_transition_from_flash_to_idle_exists);
    RUN_TEST(test_transition_from_calibration_to_idle_exists);
    RUN_TEST(test_invalid_transitions_are_null);
    RUN_TEST(test_do_init_returns_valid_state);
    RUN_TEST(test_do_idle_returns_valid_state);
    RUN_TEST(test_do_fatal_returns_valid_state);
    RUN_TEST(test_do_flash_returns_valid_state);
    RUN_TEST(test_do_calibration_returns_valid_state);

    return UNITY_END();
}
