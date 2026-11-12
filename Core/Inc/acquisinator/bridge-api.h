#ifndef BRIDGE_API_H
#define BRIDGE_API_H

#include <stdbool.h>
#include <stdint.h>

#include "bridge.h"
#include "exc-runner.h"

// Convert one lock-in window into the bridge reading and its flags.
void bridge_api_convert(const struct BridgeConfig *cfg, const struct LockinResult *res, struct BridgeReading *out);

// 0x7A0 ACQUISINATOR_DIAG: lockin_out f32 [V], adc_raw u16 (mean), adc_poll u8 (1 = SPI error), exc_id u8
void bridge_api_pack_diag(const struct LockinResult *res, const struct BridgeReading *reading, uint8_t data[8]);

// 0x7A1 ACQUISINATOR_BRIDGE: bridge_out f32 [mV/V], phase i16 [0.01 deg], flags u8, seq u8
void bridge_api_pack_bridge(const struct BridgeReading *reading, uint8_t seq, uint8_t data[8]);

// 0x7A1 with no window completed: bridge_out NaN, NO_DATA (+ SPI_ERROR) in the flags
void bridge_api_pack_no_data(bool spi_error, uint8_t data[8]);

#endif /* BRIDGE_API_H */
