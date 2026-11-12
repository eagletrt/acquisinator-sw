#include "bridge-api.h"

#include <math.h>
#include <string.h>

#define FRAME_LEN 8U
#define FRAME_STATUS 6U // flags (0x7A1) or SPI status (0x7A0)
#define FRAME_LAST 7U   // seq (0x7A1) or exc_id (0x7A0)
#define MV_PER_V 1000.0F
#define ADC_MAX_COUNTS 65535.0F

void bridge_api_convert(const struct BridgeConfig *cfg, const struct LockinResult *res, struct BridgeReading *out) {
    float i_v = res->i * cfg->adc_lsb_v;
    float q_v = res->q * cfg->adc_lsb_v;
    out->x_v = i_v * cosf(cfg->cal_phase_rad) + q_v * sinf(cfg->cal_phase_rad); // signed in-phase component
    out->phase_deg = atan2f(q_v, i_v) * 180.0F / (float)M_PI;
    out->out_mv_per_v = MV_PER_V * out->x_v / (cfg->amp_gain * res->amp_v);

    uint8_t flags = 0;
    if ((float)res->min * cfg->adc_lsb_v < cfg->clip_low_v) {
        flags |= BRIDGE_FLAG_CLIP_LOW;
    }
    if ((float)res->max * cfg->adc_lsb_v > cfg->clip_high_v) {
        flags |= BRIDGE_FLAG_CLIP_HIGH;
    }
    if (res->spi_error) {
        flags |= BRIDGE_FLAG_SPI_ERROR;
    }
    if (res->overrun) {
        flags |= BRIDGE_FLAG_OVERRUN;
    }
    out->flags = flags;
}

void bridge_api_pack_diag(const struct LockinResult *res, const struct BridgeReading *reading, uint8_t data[8]) {
    float mean = res->dc < 0.0F ? 0.0F : (res->dc > ADC_MAX_COUNTS ? ADC_MAX_COUNTS : res->dc);
    uint16_t dc_code = (uint16_t)lroundf(mean);
    memset(data, 0, FRAME_LEN);
    memcpy(&data[0], &reading->x_v, sizeof(reading->x_v)); // little endian, like the DBC
    data[4] = (uint8_t)dc_code;
    data[5] = (uint8_t)(dc_code >> 8);
    data[FRAME_STATUS] = res->spi_error ? 1U : 0U;
    data[FRAME_LAST] = res->exc_id;
}

void bridge_api_pack_bridge(const struct BridgeReading *reading, uint8_t seq, uint8_t data[8]) {
    int16_t phase_cdeg = (int16_t)lroundf(reading->phase_deg * 100.0F);
    memset(data, 0, FRAME_LEN);
    memcpy(&data[0], &reading->out_mv_per_v, sizeof(reading->out_mv_per_v));
    data[4] = (uint8_t)phase_cdeg;
    data[5] = (uint8_t)((uint16_t)phase_cdeg >> 8);
    data[FRAME_STATUS] = reading->flags;
    data[FRAME_LAST] = seq;
}

void bridge_api_pack_no_data(bool spi_error, uint8_t data[8]) {
    float none = NAN;
    memset(data, 0, FRAME_LEN);
    memcpy(&data[0], &none, sizeof(none));
    data[FRAME_STATUS] = (uint8_t)(BRIDGE_FLAG_NO_DATA | (spi_error ? BRIDGE_FLAG_SPI_ERROR : 0U));
}
