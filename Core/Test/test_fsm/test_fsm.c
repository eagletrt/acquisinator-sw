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
    TEST_ASSERT_EQUAL_STRING_MESSAGE("init", state_names[STATE_INIT], "State name for STATE_INIT is incorrect");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("idle", state_names[STATE_IDLE], "State name for STATE_IDLE is incorrect");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("fatal", state_names[STATE_FATAL], "State name for STATE_FATAL is incorrect");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("flash", state_names[STATE_FLASH], "State name for STATE_FLASH is incorrect");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("calibration", state_names[STATE_CALIBRATION], "State name for STATE_CALIBRATION is incorrect");
}

void test_init_state_transitions_to_idle(void) {
    state_t next_state = run_state(STATE_INIT, NULL);
    TEST_ASSERT_EQUAL_MESSAGE(STATE_IDLE, next_state, "INIT state did not transition to IDLE as expected");
}

void test_idle_state_stays_idle(void) {
    state_t next_state = run_state(STATE_IDLE, NULL);
    TEST_ASSERT_EQUAL_MESSAGE(STATE_IDLE, next_state, "IDLE state did not stay in IDLE as expected");
}

void test_fatal_state_stays_fatal(void) {
    state_t next_state = run_state(STATE_FATAL, NULL);
    TEST_ASSERT_EQUAL_MESSAGE(STATE_FATAL, next_state, "FATAL state did not stay in FATAL as expected");
}

void test_flash_state_stays_flash(void) {
    state_t next_state = run_state(STATE_FLASH, NULL);
    TEST_ASSERT_EQUAL_MESSAGE(STATE_FLASH, next_state, "FLASH state did not stay in FLASH as expected");
}

void test_calibration_state_stays_calibration(void) {
    state_t next_state = run_state(STATE_CALIBRATION, NULL);
    TEST_ASSERT_EQUAL_MESSAGE(STATE_CALIBRATION, next_state, "CALIBRATION state did not stay in CALIBRATION as expected");
}

void test_state_table_has_all_state_functions(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(state_table[STATE_INIT], "State function for STATE_INIT is NULL");
    TEST_ASSERT_NOT_NULL_MESSAGE(state_table[STATE_IDLE], "State function for STATE_IDLE is NULL");
    TEST_ASSERT_NOT_NULL_MESSAGE(state_table[STATE_FATAL], "State function for STATE_FATAL is NULL");
    TEST_ASSERT_NOT_NULL_MESSAGE(state_table[STATE_FLASH], "State function for STATE_FLASH is NULL");
    TEST_ASSERT_NOT_NULL_MESSAGE(state_table[STATE_CALIBRATION], "State function for STATE_CALIBRATION is NULL");
}

void test_transition_from_init_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_INIT][STATE_IDLE], "Transition function from STATE_INIT to STATE_IDLE is NULL");
}

void test_transition_from_init_to_fatal_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_INIT][STATE_FATAL], "Transition function from STATE_INIT to STATE_FATAL is NULL");
}

void test_transition_from_idle_to_flash_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_IDLE][STATE_FLASH], "Transition function from STATE_IDLE to STATE_FLASH is NULL");
}

void test_transition_from_idle_to_calibration_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_IDLE][STATE_CALIBRATION], "Transition function from STATE_IDLE to STATE_CALIBRATION is NULL");
}

void test_transition_from_flash_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_FLASH][STATE_IDLE], "Transition function from STATE_FLASH to STATE_IDLE is NULL");
}

void test_transition_from_calibration_to_idle_exists(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(transition_table[STATE_CALIBRATION][STATE_IDLE], "Transition function from STATE_CALIBRATION to STATE_IDLE is NULL");
}

void test_invalid_transitions_are_null(void) {
    TEST_ASSERT_NULL_MESSAGE(transition_table[STATE_INIT][STATE_INIT], "Transition function from STATE_INIT to STATE_INIT should be NULL");
    TEST_ASSERT_NULL_MESSAGE(transition_table[STATE_FATAL][STATE_IDLE], "Transition function from STATE_FATAL to STATE_IDLE should be NULL");
    TEST_ASSERT_NULL_MESSAGE(transition_table[STATE_FLASH][STATE_CALIBRATION], "Transition function from STATE_FLASH to STATE_CALIBRATION should be NULL");
}

void test_do_init_returns_valid_state(void) {
    state_t result = do_init(NULL);
    TEST_ASSERT_TRUE_MESSAGE(result == STATE_IDLE || result == STATE_FATAL, "do_init returned an invalid state");
}

void test_do_idle_returns_valid_state(void) {
    state_t result = do_idle(NULL);
    TEST_ASSERT_TRUE_MESSAGE(result == NO_CHANGE || result == STATE_IDLE ||
                                 result == STATE_FATAL || result == STATE_FLASH ||
                                 result == STATE_CALIBRATION,
                             "do_idle returned an invalid state");
}

void test_do_fatal_returns_valid_state(void) {
    state_t result = do_fatal(NULL);
    TEST_ASSERT_TRUE_MESSAGE(result == NO_CHANGE || result == STATE_FATAL, "do_fatal returned an invalid state");
}

void test_do_flash_returns_valid_state(void) {
    state_t result = do_flash(NULL);
    TEST_ASSERT_TRUE_MESSAGE(result == NO_CHANGE || result == STATE_IDLE ||
                                 result == STATE_FATAL || result == STATE_FLASH,
                             "do_flash returned an invalid state");
}

void test_do_calibration_returns_valid_state(void) {
    state_t result = do_calibration(NULL);
    TEST_ASSERT_TRUE_MESSAGE(result == NO_CHANGE || result == STATE_IDLE ||
                                 result == STATE_FATAL || result == STATE_CALIBRATION,
                             "do_calibration returned an invalid state");
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
