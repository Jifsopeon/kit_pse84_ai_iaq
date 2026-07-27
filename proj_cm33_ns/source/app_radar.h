#ifndef APP_RADAR_H
#define APP_RADAR_H

#include <stdbool.h>
#include <stdint.h>

#include "cy_result.h"

typedef struct
{
    bool initialized;
    bool running;
    bool presence_detected;
    bool distance_valid;
    float distance_m;
    float presence_score;
    uint32_t frame_counter;
    uint32_t acquisition_error_count;
    uint32_t processing_error_count;
} radar_result_t;

cy_rslt_t radar_init(void);
cy_rslt_t radar_start(void);
cy_rslt_t radar_process(void);
const radar_result_t *radar_get_latest_result(void);
void radar_stop(void);

#endif
