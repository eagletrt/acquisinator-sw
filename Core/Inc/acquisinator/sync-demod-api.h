#ifndef SYNC_DEMOD_API_H
#define SYNC_DEMOD_API_H

#include <stdbool.h>
#include <stdint.h>

#include "sync-demod.h"

// Fill one period of Q15 tables: sin_q15[k] = round(32767*sin(2*pi*k/period)),
// cos_q15[k] likewise. The same sin table can then drive the DAC.
void sync_demod_api_make_tables(int16_t *sin_q15, int16_t *cos_q15, uint32_t period);

// Bind the tables and set the window to `periods_per_window` whole periods
// (at most SYNC_DEMOD_MAX_PERIODS). By default the windows do not overlap.
void sync_demod_api_init(struct SyncDemodHandler *hdemod, const int16_t *sin_q15, const int16_t *cos_q15, uint32_t period, uint32_t periods_per_window);

// Sliding window: produce a result every `periods` periods, each over the last
// periods_per_window periods (periods == periods_per_window: no overlap).
void sync_demod_api_set_output_every(struct SyncDemodHandler *hdemod, uint32_t periods);

// Accumulate one sample taken at excitation phase index `phase` (0..period-1).
// Returns true when this sample completes a result: the result fields (i, q,
// dc, min, max) are then updated.
bool sync_demod_api_update(struct SyncDemodHandler *hdemod, uint16_t sample, uint32_t phase);

// Drop the running window (results of the last completed window are kept).
void sync_demod_api_reset(struct SyncDemodHandler *hdemod);

#endif /* SYNC_DEMOD_API_H */
