#ifndef EXC_PLAN_API_H
#define EXC_PLAN_API_H

#include <stdbool.h>
#include <stdint.h>

#include "exc-plan.h"

// Timing for `freq_hz`: the most samples per period the tick allows, the rounded timer
// period, and the shortest window of at least min_window_ms that cancels the mains.
// Returns false (plan untouched) when no window fits the limits.
bool exc_plan_api_make(const struct ExcPlanConfig *cfg, uint32_t freq_hz, struct ExcPlan *plan);

// Excitation levels the DAC can produce: amplitude > 0 and dc +- amplitude within out_min..out_max.
bool exc_plan_api_levels_valid(float dc_v, float amp_v, float out_min_v, float out_max_v);

// One period of DAC codes for dc + amp * sin, from the Q15 sin table of that period.
void exc_plan_api_make_dac_table(uint16_t *dac, const int16_t *sin_q15, uint32_t samples, float dc_v, float amp_v,
                                 float vdda_v, uint16_t dac_max);

#endif /* EXC_PLAN_API_H */
