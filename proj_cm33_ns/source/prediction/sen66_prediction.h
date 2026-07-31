#ifndef SEN66_PREDICTION_H
#define SEN66_PREDICTION_H

#include <stdbool.h>
#include <stdint.h>

#define SEN66_PREDICTION_NO_SMOKING   (0U)
#define SEN66_PREDICTION_SMOKING      (1U)
#define SEN66_PREDICTION_NOT_READY    (0xFFU)

void sen66_prediction_init(void);
bool sen66_prediction_update(uint16_t pm1,
                             uint16_t pm25,
                             uint16_t pm4,
                             uint16_t pm10,
                             int16_t voc,
                             int16_t nox,
                             uint16_t co2,
                             uint8_t *prediction);
uint8_t sen66_prediction_get_latest(void);
const char *sen66_prediction_format(uint8_t prediction);

#endif
