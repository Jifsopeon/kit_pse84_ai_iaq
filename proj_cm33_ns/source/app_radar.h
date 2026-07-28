#ifndef APP_RADAR_H
#define APP_RADAR_H

#include <stdbool.h>
#include <stdint.h>

#include "cy_result.h"

typedef enum
{
    RADAR_STATE_STOPPED = 0,
    RADAR_STATE_STARTING,
    RADAR_STATE_WAIT_FIFO,
    RADAR_STATE_READING_FIFO,
    RADAR_STATE_VALIDATING_FRAME,
    RADAR_STATE_PROCESSING_FRAME,
    RADAR_STATE_RECOVERING
} radar_state_t;

typedef struct
{
    bool initialized;
    bool running;
    bool presence_detected;
    bool distance_valid;
    float distance_m;
    float presence_score;
    uint32_t frame_counter;
    uint32_t task_loop_counter;
    uint32_t fifo_ready_count;
    uint32_t fifo_read_count;
    uint32_t fifo_recovery_count;
    uint32_t reinit_recovery_count;
    uint32_t spi_abort_count;
    uint32_t invalid_frame_count;
    uint32_t zero_frame_count;
    uint32_t acquisition_error_count;
    uint32_t processing_error_count;
    uint32_t frame_generation;
    uint32_t last_valid_timestamp_ms;
    uint32_t last_frame_age_ms;
    uint32_t task_stack_high_water_words;
    radar_state_t state;
} radar_result_t;

cy_rslt_t radar_init(void);
cy_rslt_t radar_start(void);
cy_rslt_t radar_process(void);
void radar_get_latest_result(radar_result_t *result);
void radar_log_watchdog(void);
void radar_stop(void);

#endif
