#ifndef ACQUISINATOR_CONFIG_H
#define ACQUISINATOR_CONFIG_H

#define ACQUISINATOR_ID 1

#define ACQUISINATORE_VREF_INT (3.29f)

#define NTC_COOLING_CONV(val)                                                  \
  (35.3762f + (232.6089 * val) - (235.0224 * pow(val, 2)) +                    \
   (85.3961 * pow(val, 3)) - (11.3187 * pow(val, 4)))

#define STRAIN_GAUGE_R1 (352)
#define STRAIN_GAUGE_R2 (350)
#define STRAIN_GAUGE_R3 (352)
#define STRAIN_GAUGE_RNOM (350)
#define STRAIN_GAUGE_G (100)
#define STRAIN_GAUGE_VREF (1.501f)
#define STRAIN_GAUGE_VS (3.294)
#define STRAIN_GAUGE_K (2.01)
#define FROM_mV_TO_ROD_ELONGATION(mVval)                                       \
  ((STRAIN_GAUGE_RNOM -                                                        \
    ((STRAIN_GAUGE_R2 * STRAIN_GAUGE_R3) -                                     \
     (STRAIN_GAUGE_R2 * ((mVval - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) *       \
      (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) /                                   \
         STRAIN_GAUGE_VS) /                                                    \
        (STRAIN_GAUGE_R1 + ((((mVval - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) *  \
                             (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) /            \
                            STRAIN_GAUGE_VS))) /                               \
   (STRAIN_GAUGE_RNOM * STRAIN_GAUGE_K))

#endif
