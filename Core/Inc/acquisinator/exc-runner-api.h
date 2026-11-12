#ifndef EXC_RUNNER_API_H
#define EXC_RUNNER_API_H

#include <stdbool.h>
#include <stdint.h>

#include "exc-runner.h"

// Default setting active, demodulator ready, first window to be dropped.
// False if the default setting is not feasible with this configuration.
bool exc_runner_api_init(struct ExcRunner *hrunner, const struct ExcRunnerConfig *cfg);

// Main loop: prepare a new setting in the free slot and hand it to the ISR. 0 = unchanged.
// False if not feasible or if the previous request has not been applied yet.
bool exc_runner_api_request(struct ExcRunner *hrunner, uint32_t freq_hz, uint32_t dc_mv, uint32_t amp_mv);

// ISR, right after the ADC sample: next DAC value, adopting a pending setting at the end
// of a period. Write step->dac (and step->autoreload if step->switched) before sampling.
void exc_runner_api_advance(struct ExcRunner *hrunner, struct ExcStep *step);

// ISR: demodulate the sample taken at this tick (sample_ok false: lost, the window restarts).
// Returns true when a window completes; `res` is then filled.
bool exc_runner_api_sample(struct ExcRunner *hrunner, const struct ExcStep *step, bool sample_ok, uint16_t raw,
                           struct LockinResult *res);

// ISR: the tick outlasted the timer period; flagged on the window in progress.
void exc_runner_api_mark_overrun(struct ExcRunner *hrunner);

// 0x7A3 ACQUISINATOR_EXC: freq u16 [0.1 Hz], samples u8, periods u8, amp u12 [mV], dc u12 [mV], id u8
void exc_runner_api_pack_status(const struct ExcSetting *exc, uint8_t data[8]);

#endif /* EXC_RUNNER_API_H */
