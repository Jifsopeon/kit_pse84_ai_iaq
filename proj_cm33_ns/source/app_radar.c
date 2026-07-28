#include "app_radar.h"

#include "app_radar_settings.h"
#include "arm_math.h"
#include "cy_scb_spi.h"
#include "cy_sysint.h"
#include "cybsp.h"
#include "ifx_sensor_dsp.h"
#include "xensiv_bgt60trxx_mtb.h"
#include "xensiv_radar_presence.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define RADAR_SPI_INTR_PRIORITY              (2U)
#define RADAR_SPI_SLAVE_SELECT              (CY_SCB_SPI_SLAVE_SELECT1)
#define RADAR_RSLT_ERR                       (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_USER_BASE, 0x01U))
#define RADAR_PI                             (3.14159265358979323846f)
#define RADAR_INTERNAL_MAX_CANDIDATES        (4U)

#if APP_RADAR_DIAGNOSTICS_ENABLED
#define RADAR_LOG(fmt, ...)                  printf("[RADAR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define RADAR_LOG(fmt, ...)                  do { } while (0)
#endif

#if APP_RADAR_VERBOSE_DIAGNOSTICS
#define RADAR_VERBOSE(fmt, ...)              printf("[RADAR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define RADAR_VERBOSE(fmt, ...)              do { } while (0)
#endif

typedef struct
{
    uint16_t start_bin;
    uint16_t end_bin;
    uint16_t peak_bin;
    float interpolated_bin;
    float distance_m;
    float strength;
    float noise_floor;
    float snr;
    uint8_t confirmation_count;
} radar_target_t;

typedef struct
{
    bool valid;
    uint8_t missed_frames;
    float filtered_distance_m;
} radar_selected_target_state_t;

typedef struct
{
    uint16_t checksum;
    uint16_t minimum;
    uint16_t maximum;
    uint32_t nonzero_count;
} radar_frame_stats_t;

static cy_stc_scb_spi_context_t radar_spi_context;
static xensiv_bgt60trxx_mtb_t radar_device;
static xensiv_radar_presence_handle_t presence_handle;
static arm_rfft_fast_instance_f32 range_fft;
static uint16_t radar_raw_frame[APP_RADAR_FRAME_SAMPLES];
static float32_t radar_float_frame[APP_RADAR_FRAME_SAMPLES];
static float32_t fft_input[APP_RADAR_RANGE_FFT_SIZE];
static float32_t fft_output[APP_RADAR_RANGE_FFT_SIZE];
static float32_t range_window[APP_RADAR_RANGE_FFT_SIZE];
static float32_t range_power[APP_RADAR_RANGE_BIN_COUNT];
static float32_t clutter_power[APP_RADAR_RANGE_BIN_COUNT];
static bool detection_bins[APP_RADAR_RANGE_BIN_COUNT];
static radar_target_t previous_targets[RADAR_INTERNAL_MAX_CANDIDATES];
static radar_selected_target_state_t selected_state;
static radar_result_t latest_result;
static float radar_bin_length_m;
static uint16_t min_range_bin;
static uint16_t max_range_bin;
static uint16_t leakage_reject_bin;
static uint32_t absence_frames;
static uint32_t stale_frame_polls;
static uint32_t consecutive_invalid_frames;
static uint8_t previous_target_count;
static bool motion_presence_detected;
static bool last_presence_state;
static bool presence_state_reported;
static TickType_t last_frame_tick;
static TickType_t last_check_log_tick;
static TickType_t last_watchdog_log_tick;

static void radar_spi_interrupt_handler(void)
{
    Cy_SCB_SPI_Interrupt(CYBSP_SPI_CONTROLLER_HW, &radar_spi_context);
}

static bool distance_is_plausible(float distance_m)
{
    return isfinite(distance_m) &&
           (distance_m >= APP_RADAR_MIN_DISTANCE_M) &&
           (distance_m <= APP_RADAR_MAX_DISTANCE_M);
}

static const char *radar_state_name(radar_state_t state)
{
    switch (state)
    {
        case RADAR_STATE_STOPPED:
            return "STOPPED";
        case RADAR_STATE_STARTING:
            return "STARTING";
        case RADAR_STATE_WAIT_FIFO:
            return "WAIT_FIFO";
        case RADAR_STATE_READING_FIFO:
            return "READING_FIFO";
        case RADAR_STATE_VALIDATING_FRAME:
            return "VALIDATING_FRAME";
        case RADAR_STATE_PROCESSING_FRAME:
            return "PROCESSING_FRAME";
        case RADAR_STATE_RECOVERING:
            return "RECOVERING";
        default:
            return "UNKNOWN";
    }
}

static void set_radar_state(radar_state_t state)
{
    latest_result.state = state;
}

static uint32_t get_radar_task_stack_high_water_words(void)
{
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    return (uint32_t)uxTaskGetStackHighWaterMark(NULL);
#else
    return 0U;
#endif
}

static uint16_t clamp_bin(int32_t bin)
{
    if (bin < 0)
    {
        return 0U;
    }
    if (bin >= (int32_t)APP_RADAR_RANGE_BIN_COUNT)
    {
        return (uint16_t)(APP_RADAR_RANGE_BIN_COUNT - 1U);
    }
    return (uint16_t)bin;
}

static void init_range_window(void)
{
    for (uint32_t i = 0U; i < APP_RADAR_RANGE_FFT_SIZE; i++)
    {
        range_window[i] = 0.5f - (0.5f * cosf((2.0f * RADAR_PI * (float)i) /
                                              (float)(APP_RADAR_RANGE_FFT_SIZE - 1U)));
    }
}

static float fft_bin_power(uint16_t bin)
{
    if (0U == bin)
    {
        return fft_output[0] * fft_output[0];
    }
    if ((APP_RADAR_RANGE_FFT_SIZE / 2U) == bin)
    {
        return fft_output[1] * fft_output[1];
    }

    float real = fft_output[2U * bin];
    float imag = fft_output[(2U * bin) + 1U];
    return (real * real) + (imag * imag);
}

static void build_range_profile(void)
{
    bool update_clutter = (latest_result.frame_counter < APP_RADAR_CLUTTER_SETTLE_FRAMES) ||
                          !motion_presence_detected;

    for (uint32_t bin = 0U; bin < APP_RADAR_RANGE_BIN_COUNT; bin++)
    {
        range_power[bin] = 0.0f;
    }

    for (uint32_t chirp = 0U; chirp < APP_RADAR_CHIRPS_PER_FRAME; chirp++)
    {
        uint32_t chirp_offset = chirp * APP_RADAR_SAMPLES_PER_CHIRP * APP_RADAR_RX_ANTENNAS;
        float mean = 0.0f;

        for (uint32_t sample = 0U; sample < APP_RADAR_SAMPLES_PER_CHIRP; sample++)
        {
            mean += radar_float_frame[chirp_offset + sample];
        }
        mean /= (float)APP_RADAR_SAMPLES_PER_CHIRP;

        for (uint32_t sample = 0U; sample < APP_RADAR_SAMPLES_PER_CHIRP; sample++)
        {
            fft_input[sample] = (radar_float_frame[chirp_offset + sample] - mean) *
                                range_window[sample];
        }

        arm_rfft_fast_f32(&range_fft, fft_input, fft_output, 0U);

        for (uint32_t bin = 0U; bin < APP_RADAR_RANGE_BIN_COUNT; bin++)
        {
            range_power[bin] += fft_bin_power((uint16_t)bin);
        }
    }

    for (uint32_t bin = 0U; bin < APP_RADAR_RANGE_BIN_COUNT; bin++)
    {
        range_power[bin] /= (float)APP_RADAR_CHIRPS_PER_FRAME;

        if (0U == latest_result.frame_counter)
        {
            clutter_power[bin] = range_power[bin];
        }
        else if (update_clutter)
        {
            clutter_power[bin] += APP_RADAR_CLUTTER_ALPHA * (range_power[bin] - clutter_power[bin]);
        }

        range_power[bin] = (range_power[bin] > clutter_power[bin]) ?
                           (range_power[bin] - clutter_power[bin]) : 0.0f;
    }
}

static float estimate_noise_floor(uint16_t center_bin)
{
    float noise_sum = 0.0f;
    uint16_t noise_count = 0U;

    for (uint16_t bin = min_range_bin; bin <= max_range_bin; bin++)
    {
        uint16_t low_guard = (center_bin > APP_RADAR_NOISE_GUARD_BINS) ?
                             (uint16_t)(center_bin - APP_RADAR_NOISE_GUARD_BINS) : 0U;
        uint16_t high_guard = (uint16_t)(center_bin + APP_RADAR_NOISE_GUARD_BINS);

        if ((bin >= low_guard) && (bin <= high_guard))
        {
            continue;
        }

        noise_sum += range_power[bin];
        noise_count++;
    }

    if (0U == noise_count)
    {
        return 1.0f;
    }

    float noise = noise_sum / (float)noise_count;
    return (noise > 1.0f) ? noise : 1.0f;
}

static float interpolate_peak_bin(uint16_t peak_bin)
{
    if ((peak_bin <= min_range_bin) || (peak_bin >= max_range_bin))
    {
        return (float)peak_bin;
    }

    float left = range_power[peak_bin - 1U];
    float center = range_power[peak_bin];
    float right = range_power[peak_bin + 1U];
    float denominator = left - (2.0f * center) + right;

    if (fabsf(denominator) < 1.0e-6f)
    {
        return (float)peak_bin;
    }

    float offset = 0.5f * (left - right) / denominator;
    if (offset > 0.5f)
    {
        offset = 0.5f;
    }
    else if (offset < -0.5f)
    {
        offset = -0.5f;
    }

    return (float)peak_bin + offset;
}

static uint8_t find_candidate_targets(radar_target_t *targets, uint8_t max_targets)
{
    uint8_t target_count = 0U;

    for (uint32_t bin = 0U; bin < APP_RADAR_RANGE_BIN_COUNT; bin++)
    {
        detection_bins[bin] = false;
    }

    if (latest_result.frame_counter < APP_RADAR_CLUTTER_SETTLE_FRAMES)
    {
        return 0U;
    }

    for (uint16_t bin = min_range_bin; bin <= max_range_bin; bin++)
    {
        if (bin < leakage_reject_bin)
        {
            continue;
        }

        float noise = estimate_noise_floor(bin);
        float snr = range_power[bin] / noise;
        float threshold = noise * APP_RADAR_THRESHOLD_SCALE;

        detection_bins[bin] = (range_power[bin] >= APP_RADAR_MIN_PEAK_POWER) &&
                              (range_power[bin] >= threshold) &&
                              (snr >= APP_RADAR_THRESHOLD_SNR);
    }

    uint16_t bin = min_range_bin;
    while (bin <= max_range_bin)
    {
        while ((bin <= max_range_bin) && !detection_bins[bin])
        {
            bin++;
        }
        if (bin > max_range_bin)
        {
            break;
        }

        uint16_t start_bin = bin;
        uint16_t end_bin = bin;
        uint8_t gap = 0U;
        while (++bin <= max_range_bin)
        {
            if (detection_bins[bin])
            {
                end_bin = bin;
                gap = 0U;
            }
            else if (gap < APP_RADAR_CLUSTER_MAX_GAP_BINS)
            {
                gap++;
            }
            else
            {
                break;
            }
        }

        if (((uint16_t)(end_bin - start_bin + 1U) < APP_RADAR_MIN_CLUSTER_WIDTH_BINS) ||
            (target_count >= max_targets))
        {
            continue;
        }

        uint16_t peak_bin = start_bin;
        float peak_power = range_power[start_bin];
        for (uint16_t candidate = start_bin + 1U; candidate <= end_bin; candidate++)
        {
            if (range_power[candidate] > peak_power)
            {
                peak_power = range_power[candidate];
                peak_bin = candidate;
            }
        }

        float noise = estimate_noise_floor(peak_bin);
        float interpolated_bin = interpolate_peak_bin(peak_bin);
        float distance_m = interpolated_bin * radar_bin_length_m;

        if (!distance_is_plausible(distance_m))
        {
            continue;
        }

        targets[target_count].start_bin = start_bin;
        targets[target_count].end_bin = end_bin;
        targets[target_count].peak_bin = peak_bin;
        targets[target_count].interpolated_bin = interpolated_bin;
        targets[target_count].distance_m = distance_m;
        targets[target_count].strength = peak_power;
        targets[target_count].noise_floor = noise;
        targets[target_count].snr = peak_power / noise;
        targets[target_count].confirmation_count = 1U;
        target_count++;
    }

    return target_count;
}

static uint8_t confirm_targets(radar_target_t *targets, uint8_t target_count)
{
    uint8_t confirmed_count = 0U;

    for (uint8_t i = 0U; i < target_count; i++)
    {
        uint8_t confirmation_count = 1U;

        for (uint8_t j = 0U; j < previous_target_count; j++)
        {
            if (fabsf(targets[i].distance_m - previous_targets[j].distance_m) <=
                (radar_bin_length_m * 1.5f))
            {
                confirmation_count = (uint8_t)(previous_targets[j].confirmation_count + 1U);
                break;
            }
        }

        targets[i].confirmation_count = confirmation_count;
        if (confirmation_count >= APP_RADAR_CONFIRMATION_FRAMES)
        {
            targets[confirmed_count++] = targets[i];
        }
    }

    return confirmed_count;
}

static void copy_current_targets(const radar_target_t *targets, uint8_t target_count)
{
    previous_target_count = target_count;
    for (uint8_t i = 0U; i < target_count; i++)
    {
        previous_targets[i] = targets[i];
    }
}

static void clear_published_distance(void)
{
    taskENTER_CRITICAL();
    latest_result.distance_valid = false;
    latest_result.distance_m = 0.0f;
    taskEXIT_CRITICAL();
}

static void publish_presence_distance(bool presence_detected, bool distance_valid, float distance_m)
{
    taskENTER_CRITICAL();
    latest_result.presence_detected = presence_detected;
    latest_result.distance_valid = distance_valid;
    latest_result.distance_m = distance_valid ? distance_m : 0.0f;
    latest_result.frame_generation++;
    if (distance_valid)
    {
        latest_result.last_valid_timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    taskEXIT_CRITICAL();
}

static void reset_tracking_state(void)
{
    selected_state.valid = false;
    selected_state.missed_frames = 0U;
    previous_target_count = 0U;
    absence_frames = 0U;
    motion_presence_detected = false;
    publish_presence_distance(false, false, 0.0f);
}

static void update_selected_distance(const radar_target_t *targets, uint8_t target_count)
{
    if (!motion_presence_detected || (0U == target_count))
    {
        if (selected_state.valid && (selected_state.missed_frames < APP_RADAR_ALLOWED_MISSED_FRAMES))
        {
            selected_state.missed_frames++;
        }
        else
        {
            selected_state.valid = false;
            clear_published_distance();
        }
        return;
    }

    const radar_target_t *nearest = &targets[0];

    for (uint8_t i = 1U; i < target_count; i++)
    {
        if (targets[i].distance_m < nearest->distance_m)
        {
            nearest = &targets[i];
        }
    }

    /* Binary presence plus one selected range estimate. If multiple FFT peaks
     * exist internally, publish only the nearest confirmed valid target.
     */
    float selected_distance_m = nearest->distance_m;
    bool target_jump = selected_state.valid &&
                       (fabsf(selected_distance_m - selected_state.filtered_distance_m) >
                        APP_RADAR_MAX_JUMP_M);

    if (!selected_state.valid || target_jump)
    {
        selected_state.filtered_distance_m = selected_distance_m;
        selected_state.valid = true;
    }
    else
    {
        selected_state.filtered_distance_m += APP_RADAR_DISTANCE_ALPHA *
                                              (selected_distance_m - selected_state.filtered_distance_m);
    }

    selected_state.missed_frames = 0U;
    publish_presence_distance(true, true, selected_state.filtered_distance_m);
    absence_frames = 0U;
}

#if APP_RADAR_VERBOSE_DIAGNOSTICS
static void log_range_diagnostics(const radar_target_t *targets, uint8_t target_count)
{
    static uint32_t last_logged_frame;
    static bool last_distance_valid;

    bool state_changed = (last_distance_valid != latest_result.distance_valid);
    bool periodic = ((latest_result.frame_counter - last_logged_frame) >=
                     APP_RADAR_DIAGNOSTIC_PERIOD_FRAMES);

    if (!state_changed && !periodic)
    {
        return;
    }

    uint16_t selected_bin = 0U;
    float peak_strength = 0.0f;
    float noise_floor = 0.0f;
    float snr = 0.0f;

    if (latest_result.distance_valid && (target_count > 0U))
    {
        const radar_target_t *nearest = &targets[0];
        for (uint8_t i = 1U; i < target_count; i++)
        {
            if (targets[i].distance_m < nearest->distance_m)
            {
                nearest = &targets[i];
            }
        }
        selected_bin = nearest->peak_bin;
        peak_strength = nearest->strength;
        noise_floor = nearest->noise_floor;
        snr = nearest->snr;
    }

    RADAR_LOG("frame=%lu presence=%u selected_rule=nearest selected_bin=%u distance_m=%.2f peak_strength=%.0f noise_floor=%.0f snr=%.1f distance_valid=%u published_distance=%u",
              (unsigned long)latest_result.frame_counter,
              latest_result.presence_detected ? 1U : 0U,
              (unsigned int)selected_bin,
              (double)latest_result.distance_m,
              (double)peak_strength,
              (double)noise_floor,
              (double)snr,
              latest_result.distance_valid ? 1U : 0U,
              latest_result.distance_valid ? (unsigned int)((latest_result.distance_m * 1000.0f) + 0.5f) : 0U);

    last_logged_frame = latest_result.frame_counter;
    last_distance_valid = latest_result.distance_valid;
}
#endif

static void presence_callback(xensiv_radar_presence_handle_t handle,
                              const xensiv_radar_presence_event_t *event,
                              void *data)
{
    CY_UNUSED_PARAMETER(handle);
    CY_UNUSED_PARAMETER(data);

    if (NULL == event)
    {
        return;
    }

    motion_presence_detected = (XENSIV_RADAR_PRESENCE_STATE_ABSENCE != event->state);
}

static void invalidate_after_absence(void)
{
    if (!motion_presence_detected)
    {
        if (absence_frames < APP_RADAR_ABSENCE_RESET_FRAMES)
        {
            absence_frames++;
        }
        if (absence_frames >= APP_RADAR_ABSENCE_RESET_FRAMES)
        {
            selected_state.valid = false;
            publish_presence_distance(false, false, 0.0f);
        }
    }
}

static radar_frame_stats_t analyze_raw_frame(void)
{
    radar_frame_stats_t stats =
    {
        .checksum = 0U,
        .minimum = UINT16_MAX,
        .maximum = 0U,
        .nonzero_count = 0U,
    };

    for (uint32_t i = 0U; i < APP_RADAR_FRAME_SAMPLES; i++)
    {
        uint16_t sample = radar_raw_frame[i];
        stats.checksum = (uint16_t)((stats.checksum << 1U) ^ sample);
        if (sample < stats.minimum)
        {
            stats.minimum = sample;
        }
        if (sample > stats.maximum)
        {
            stats.maximum = sample;
        }
        if (0U != sample)
        {
            stats.nonzero_count++;
        }
    }

    return stats;
}

static bool frame_stats_are_valid(const radar_frame_stats_t *stats)
{
    uint16_t span = (uint16_t)(stats->maximum - stats->minimum);

    return (stats->nonzero_count >= APP_RADAR_MIN_NONZERO_SAMPLES) &&
           (span >= APP_RADAR_MIN_RAW_SPAN);
}

static cy_rslt_t reinitialize_radar_stream(const char *reason);

static cy_rslt_t restart_radar_fifo_stream(const char *reason)
{
    int32_t reset_status;
    int32_t start_status;

    set_radar_state(RADAR_STATE_RECOVERING);
    latest_result.fifo_recovery_count++;
    reset_tracking_state();

    (void)xensiv_bgt60trxx_start_frame(&radar_device.dev, false);
    reset_status = xensiv_bgt60trxx_soft_reset(&radar_device.dev, XENSIV_BGT60TRXX_RESET_FIFO);
    Cy_GPIO_ClearInterrupt(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_PIN);
    start_status = xensiv_bgt60trxx_start_frame(&radar_device.dev, true);
    stale_frame_polls = 0U;
    consecutive_invalid_frames = 0U;

    if ((XENSIV_BGT60TRXX_STATUS_OK != reset_status) ||
        (XENSIV_BGT60TRXX_STATUS_OK != start_status))
    {
        latest_result.acquisition_error_count++;
        latest_result.running = false;
        RADAR_LOG("stream recovery failed reason=%s reset=%ld start=%ld recoveries=%lu",
                  reason,
                  (long)reset_status,
                  (long)start_status,
                  (unsigned long)latest_result.fifo_recovery_count);
        if ((latest_result.fifo_recovery_count % APP_RADAR_REINIT_RECOVERY_LIMIT) == 0U)
        {
            return reinitialize_radar_stream(reason);
        }
        return RADAR_RSLT_ERR;
    }

    latest_result.running = true;
    last_frame_tick = xTaskGetTickCount();
    set_radar_state(RADAR_STATE_WAIT_FIFO);
    RADAR_LOG("stream recovered reason=%s recoveries=%lu",
              reason,
              (unsigned long)latest_result.fifo_recovery_count);
    return CY_RSLT_SUCCESS;
}

static cy_rslt_t reinitialize_radar_stream(const char *reason)
{
    cy_rslt_t result;

    set_radar_state(RADAR_STATE_RECOVERING);
    latest_result.reinit_recovery_count++;
    latest_result.running = false;
    reset_tracking_state();

    (void)xensiv_bgt60trxx_start_frame(&radar_device.dev, false);
    result = xensiv_bgt60trxx_mtb_init(&radar_device,
                                       app_radar_register_list,
                                       APP_RADAR_REGISTER_COUNT);
    if (CY_RSLT_SUCCESS == result)
    {
        result = xensiv_bgt60trxx_mtb_interrupt_init(&radar_device,
                                                     APP_RADAR_FIFO_LIMIT_SAMPLES);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = radar_start();
    }

    RADAR_LOG("reinit recovery reason=%s result=0x%08lx reinit=%lu",
              reason,
              (unsigned long)result,
              (unsigned long)latest_result.reinit_recovery_count);
    return result;
}

static void log_radar_liveness(uint16_t peak_bin,
                               float raw_distance_m,
                               float filtered_distance_m,
                               bool distance_valid,
                               uint8_t candidate_count,
                               uint8_t confirmed_count,
                               uint16_t raw_checksum)
{
    TickType_t now = xTaskGetTickCount();

    if ((now - last_check_log_tick) < pdMS_TO_TICKS(1000U))
    {
        return;
    }

    last_check_log_tick = now;
    latest_result.task_stack_high_water_words = get_radar_task_stack_high_water_words();
    RADAR_LOG("CHECK loop=%lu frame=%lu fifo_ready=%lu fifo_read=%lu recover=%lu reinit=%lu abort=%lu invalid=%lu zero=%lu age_ms=%lu state=%s stack=%lu peak=%u raw=%.2f filtered=%.2f valid=%u presence=%u candidates=%u confirmed=%u missed=%u checksum=0x%04X",
              (unsigned long)latest_result.task_loop_counter,
              (unsigned long)latest_result.frame_counter,
              (unsigned long)latest_result.fifo_ready_count,
              (unsigned long)latest_result.fifo_read_count,
              (unsigned long)latest_result.fifo_recovery_count,
              (unsigned long)latest_result.reinit_recovery_count,
              (unsigned long)latest_result.spi_abort_count,
              (unsigned long)latest_result.invalid_frame_count,
              (unsigned long)latest_result.zero_frame_count,
              (unsigned long)((now - last_frame_tick) * portTICK_PERIOD_MS),
              radar_state_name(latest_result.state),
              (unsigned long)latest_result.task_stack_high_water_words,
              (unsigned int)peak_bin,
              (double)raw_distance_m,
              (double)filtered_distance_m,
              distance_valid ? 1U : 0U,
              latest_result.presence_detected ? 1U : 0U,
              (unsigned int)candidate_count,
              (unsigned int)confirmed_count,
              (unsigned int)selected_state.missed_frames,
              (unsigned int)raw_checksum);
}

cy_rslt_t radar_init(void)
{
    cy_rslt_t result;

    if (latest_result.initialized)
    {
        return CY_RSLT_SUCCESS;
    }

    result = (cy_rslt_t)Cy_SCB_SPI_Init(CYBSP_SPI_CONTROLLER_HW,
                                        &CYBSP_SPI_CONTROLLER_config,
                                        &radar_spi_context);
    if (CY_RSLT_SUCCESS != result)
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("SPI init failed result=0x%08lx", (unsigned long)result);
        return result;
    }

    cy_stc_sysint_t spi_intr_config =
    {
        .intrSrc = (IRQn_Type)CYBSP_SPI_CONTROLLER_IRQ,
        .intrPriority = RADAR_SPI_INTR_PRIORITY
    };

    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&spi_intr_config, radar_spi_interrupt_handler))
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("SPI interrupt init failed");
        return RADAR_RSLT_ERR;
    }

    NVIC_EnableIRQ(spi_intr_config.intrSrc);
    Cy_SCB_SPI_SetActiveSlaveSelect(CYBSP_SPI_CONTROLLER_HW, RADAR_SPI_SLAVE_SELECT);
    Cy_SCB_SPI_Enable(CYBSP_SPI_CONTROLLER_HW);

    radar_device.iface.scb_inst = CYBSP_SPI_CONTROLLER_HW;
    radar_device.iface.spi = &radar_spi_context;
    radar_device.iface.sel_port = CYBSP_RSPI_CS_PORT;
    radar_device.iface.sel_pin = CYBSP_RSPI_CS_PIN;
    radar_device.iface.rst_port = CYBSP_RADAR_RESET_PORT;
    radar_device.iface.rst_pin = CYBSP_RADAR_RESET_PIN;
    radar_device.iface.irq_port = CYBSP_RADAR_INT_PORT;
    radar_device.iface.irq_pin = CYBSP_RADAR_INT_PIN;
    radar_device.iface.irq_num = CYBSP_RADAR_INT_IRQ;

    Cy_GPIO_SetSlewRate(CYBSP_RSPI_MOSI_PORT, CYBSP_RSPI_MOSI_PIN, CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYBSP_RSPI_MOSI_PORT, CYBSP_RSPI_MOSI_PIN, CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYBSP_RSPI_CLK_PORT, CYBSP_RSPI_CLK_PIN, CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYBSP_RSPI_CLK_PORT, CYBSP_RSPI_CLK_PIN, CY_GPIO_DRIVE_1_8);

    result = xensiv_bgt60trxx_mtb_init(&radar_device,
                                       app_radar_register_list,
                                       APP_RADAR_REGISTER_COUNT);
    if (CY_RSLT_SUCCESS != result)
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("sensor init failed result=0x%08lx", (unsigned long)result);
        return result;
    }

    result = xensiv_bgt60trxx_mtb_interrupt_init(&radar_device,
                                                 APP_RADAR_FIFO_LIMIT_SAMPLES);
    if (CY_RSLT_SUCCESS != result)
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("sensor interrupt config failed result=0x%08lx", (unsigned long)result);
        return result;
    }

    if (ARM_MATH_SUCCESS != arm_rfft_fast_init_f32(&range_fft, APP_RADAR_RANGE_FFT_SIZE))
    {
        latest_result.processing_error_count++;
        RADAR_LOG("range FFT init failed");
        return RADAR_RSLT_ERR;
    }

    init_range_window();
    radar_bin_length_m = ifx_range_resolution(APP_RADAR_BANDWIDTH_HZ);
    min_range_bin = clamp_bin((int32_t)ceilf(APP_RADAR_MIN_DISTANCE_M / radar_bin_length_m));
    max_range_bin = clamp_bin((int32_t)floorf(APP_RADAR_MAX_DISTANCE_M / radar_bin_length_m));
    leakage_reject_bin = clamp_bin((int32_t)ceilf(APP_RADAR_LEAKAGE_REJECT_DISTANCE_M / radar_bin_length_m));

    xensiv_radar_presence_set_malloc_free(malloc, free);

    xensiv_radar_presence_config_t presence_config;
    xensiv_radar_presence_init_config(&presence_config);
    presence_config.num_samples_per_chirp = APP_RADAR_SAMPLES_PER_CHIRP;
    presence_config.micro_fft_size = 128;
    presence_config.bandwidth = APP_RADAR_BANDWIDTH_HZ;
    presence_config.min_range_bin = min_range_bin;
    presence_config.max_range_bin = max_range_bin;
    presence_config.macro_movement_confirmations = 3;
    presence_config.macro_movement_validity_ms = 1000U;
    presence_config.micro_movement_validity_ms = 4000U;
    presence_config.mode = XENSIV_RADAR_PRESENCE_MODE_MICRO_IF_MACRO;

    int32_t presence_status = xensiv_radar_presence_alloc(&presence_handle, &presence_config);
    if (XENSIV_RADAR_PRESENCE_OK != presence_status)
    {
        latest_result.processing_error_count++;
        RADAR_LOG("presence alloc failed status=%ld", (long)presence_status);
        return RADAR_RSLT_ERR;
    }

    xensiv_radar_presence_set_callback(presence_handle, presence_callback, NULL);

    latest_result.initialized = true;
    last_frame_tick = xTaskGetTickCount();
    last_check_log_tick = last_frame_tick;
    last_watchdog_log_tick = last_frame_tick;
    set_radar_state(RADAR_STATE_STOPPED);
    RADAR_LOG("initialized BGT60TR13C frame_samples=%u fft=%u bin=%.3f m min_bin=%u max_bin=%u leakage_bin=%u",
              (unsigned int)APP_RADAR_FRAME_SAMPLES,
              (unsigned int)APP_RADAR_RANGE_FFT_SIZE,
              (double)radar_bin_length_m,
              (unsigned int)min_range_bin,
              (unsigned int)max_range_bin,
              (unsigned int)leakage_reject_bin);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t radar_start(void)
{
    if (!latest_result.initialized)
    {
        return RADAR_RSLT_ERR;
    }

    set_radar_state(RADAR_STATE_STARTING);
    if (XENSIV_BGT60TRXX_STATUS_OK !=
        xensiv_bgt60trxx_soft_reset(&radar_device.dev, XENSIV_BGT60TRXX_RESET_FIFO))
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("FIFO reset failed");
        return RADAR_RSLT_ERR;
    }

    if (XENSIV_BGT60TRXX_STATUS_OK != xensiv_bgt60trxx_start_frame(&radar_device.dev, true))
    {
        latest_result.acquisition_error_count++;
        RADAR_LOG("start frame failed");
        return RADAR_RSLT_ERR;
    }

    latest_result.running = true;
    stale_frame_polls = 0U;
    consecutive_invalid_frames = 0U;
    reset_tracking_state();
    last_frame_tick = xTaskGetTickCount();
    set_radar_state(RADAR_STATE_WAIT_FIFO);
    RADAR_LOG("started");
    return CY_RSLT_SUCCESS;
}

cy_rslt_t radar_process(void)
{
    if (!latest_result.running)
    {
        set_radar_state(RADAR_STATE_STOPPED);
        return RADAR_RSLT_ERR;
    }

    latest_result.task_loop_counter++;
    latest_result.task_stack_high_water_words = get_radar_task_stack_high_water_words();
    set_radar_state(RADAR_STATE_WAIT_FIFO);

    if (0U == Cy_GPIO_Read(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_PIN))
    {
        if (stale_frame_polls < APP_RADAR_STALE_RECOVERY_POLLS)
        {
            stale_frame_polls++;
        }
        if (stale_frame_polls >= APP_RADAR_STALE_FRAME_POLLS)
        {
            reset_tracking_state();
            if (stale_frame_polls >= APP_RADAR_STALE_RECOVERY_POLLS)
            {
                return restart_radar_fifo_stream("stale_fifo_ready");
            }
        }
        invalidate_after_absence();
        log_radar_liveness(0U, 0.0f, 0.0f, false, 0U, 0U, 0U);
        return CY_RSLT_SUCCESS;
    }
    stale_frame_polls = 0U;
    latest_result.fifo_ready_count++;

    uint32_t fifo_status_before = 0U;
    (void)xensiv_bgt60trxx_get_fifo_status(&radar_device.dev, &fifo_status_before);
    set_radar_state(RADAR_STATE_READING_FIFO);
    int32_t status = xensiv_bgt60trxx_get_fifo_data(&radar_device.dev,
                                                    radar_raw_frame,
                                                    APP_RADAR_FRAME_SAMPLES);
    Cy_GPIO_ClearInterrupt(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_PIN);
    uint32_t fifo_status_after = 0U;
    (void)xensiv_bgt60trxx_get_fifo_status(&radar_device.dev, &fifo_status_after);

    if (XENSIV_BGT60TRXX_STATUS_OK != status)
    {
        latest_result.acquisition_error_count++;
        clear_published_distance();
        RADAR_LOG("FIFO read failed status=%ld", (long)status);
        return RADAR_RSLT_ERR;
    }

    set_radar_state(RADAR_STATE_VALIDATING_FRAME);
    radar_frame_stats_t frame_stats = analyze_raw_frame();
    if (!frame_stats_are_valid(&frame_stats))
    {
        latest_result.invalid_frame_count++;
        if (0U == frame_stats.nonzero_count)
        {
            latest_result.zero_frame_count++;
        }
        consecutive_invalid_frames++;
        reset_tracking_state();
        RADAR_LOG("IO invalid_frame fifo_before=0x%06lX fifo_after=0x%06lX requested_samples=%u requested_bytes=%u buffer=0x%08lX min=%u max=%u nonzero=%lu checksum=0x%04X consecutive=%lu",
                  (unsigned long)fifo_status_before,
                  (unsigned long)fifo_status_after,
                  (unsigned int)APP_RADAR_FRAME_SAMPLES,
                  (unsigned int)(APP_RADAR_FRAME_SAMPLES * sizeof(radar_raw_frame[0])),
                  (unsigned long)(uintptr_t)radar_raw_frame,
                  (unsigned int)frame_stats.minimum,
                  (unsigned int)frame_stats.maximum,
                  (unsigned long)frame_stats.nonzero_count,
                  (unsigned int)frame_stats.checksum,
                  (unsigned long)consecutive_invalid_frames);
        if (consecutive_invalid_frames >= APP_RADAR_INVALID_FRAME_RECOVERY_COUNT)
        {
            return restart_radar_fifo_stream("invalid_frame");
        }
        return CY_RSLT_SUCCESS;
    }

    consecutive_invalid_frames = 0U;
    latest_result.fifo_read_count++;
    last_frame_tick = xTaskGetTickCount();
    for (uint32_t i = 0U; i < APP_RADAR_FRAME_SAMPLES; i++)
    {
        radar_float_frame[i] = (float32_t)((int16_t)radar_raw_frame[i]);
    }

    set_radar_state(RADAR_STATE_PROCESSING_FRAME);
    build_range_profile();

    int32_t presence_status = xensiv_radar_presence_process_frame(presence_handle,
                                                                  radar_float_frame,
                                                                  xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (XENSIV_RADAR_PRESENCE_OK != presence_status)
    {
        latest_result.processing_error_count++;
        clear_published_distance();
        RADAR_LOG("presence process failed status=%ld", (long)presence_status);
        return RADAR_RSLT_ERR;
    }

    float macro = 0.0f;
    float micro = 0.0f;
    int macro_index = 0;
    int micro_index = 0;

    latest_result.presence_score = 0.0f;
    if (xensiv_radar_presence_get_max_macro(presence_handle, &macro, &macro_index))
    {
        latest_result.presence_score = macro;
        RADAR_VERBOSE("macro score=%.3f bin=%d", (double)macro, macro_index);
    }
    if (xensiv_radar_presence_get_max_micro(presence_handle, &micro, &micro_index) &&
        (micro > latest_result.presence_score))
    {
        latest_result.presence_score = micro;
        RADAR_VERBOSE("micro score=%.3f bin=%d", (double)micro, micro_index);
    }

    radar_target_t candidate_targets[RADAR_INTERNAL_MAX_CANDIDATES];
    uint8_t candidate_count = find_candidate_targets(candidate_targets, RADAR_INTERNAL_MAX_CANDIDATES);
    uint8_t confirmed_count = confirm_targets(candidate_targets, candidate_count);
    uint16_t raw_peak_bin = 0U;
    float raw_distance_m = 0.0f;
    if (confirmed_count > 0U)
    {
        const radar_target_t *nearest = &candidate_targets[0];
        for (uint8_t i = 1U; i < confirmed_count; i++)
        {
            if (candidate_targets[i].distance_m < nearest->distance_m)
            {
                nearest = &candidate_targets[i];
            }
        }
        raw_peak_bin = nearest->peak_bin;
        raw_distance_m = nearest->distance_m;
    }

    latest_result.frame_counter++;
    update_selected_distance(candidate_targets, confirmed_count);
    copy_current_targets(candidate_targets, candidate_count);
    invalidate_after_absence();

    if (!presence_state_reported || (latest_result.presence_detected != last_presence_state))
    {
        RADAR_LOG("Presence=%u", latest_result.presence_detected ? 1U : 0U);
        last_presence_state = latest_result.presence_detected;
        presence_state_reported = true;
    }

#if APP_RADAR_VERBOSE_DIAGNOSTICS
    log_range_diagnostics(candidate_targets, confirmed_count);
#endif
    log_radar_liveness(raw_peak_bin,
                       raw_distance_m,
                       latest_result.distance_m,
                       latest_result.distance_valid,
                       candidate_count,
                       confirmed_count,
                       frame_stats.checksum);
    set_radar_state(RADAR_STATE_WAIT_FIFO);

    return CY_RSLT_SUCCESS;
}

void radar_get_latest_result(radar_result_t *result)
{
    if (NULL == result)
    {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t frame_tick;

    taskENTER_CRITICAL();
    *result = latest_result;
    frame_tick = last_frame_tick;
    taskEXIT_CRITICAL();

    result->last_frame_age_ms = (uint32_t)((now - frame_tick) * portTICK_PERIOD_MS);
}

void radar_log_watchdog(void)
{
#if APP_RADAR_DIAGNOSTICS_ENABLED
    TickType_t now = xTaskGetTickCount();

    if ((now - last_watchdog_log_tick) < pdMS_TO_TICKS(APP_RADAR_WATCHDOG_PERIOD_MS))
    {
        return;
    }

    if ((now - last_check_log_tick) < pdMS_TO_TICKS(APP_RADAR_WATCHDOG_PERIOD_MS))
    {
        return;
    }

    last_watchdog_log_tick = now;

    radar_result_t snapshot;
    radar_get_latest_result(&snapshot);
    if ((RADAR_STATE_READING_FIFO == snapshot.state) &&
        (snapshot.last_frame_age_ms >= APP_RADAR_SPI_STALL_ABORT_MS))
    {
        latest_result.spi_abort_count++;
        Cy_SCB_SPI_AbortTransfer(CYBSP_SPI_CONTROLLER_HW, &radar_spi_context);
        Cy_GPIO_ClearInterrupt(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_PIN);
        RADAR_LOG("WATCH abort_spi age_ms=%lu abort=%lu",
                  (unsigned long)snapshot.last_frame_age_ms,
                  (unsigned long)latest_result.spi_abort_count);
    }

    RADAR_LOG("WATCH loop=%lu frame=%lu fifo_ready=%lu fifo_read=%lu recover=%lu reinit=%lu abort=%lu invalid=%lu zero=%lu age_ms=%lu state=%s stack=%lu",
              (unsigned long)snapshot.task_loop_counter,
              (unsigned long)snapshot.frame_counter,
              (unsigned long)snapshot.fifo_ready_count,
              (unsigned long)snapshot.fifo_read_count,
              (unsigned long)snapshot.fifo_recovery_count,
              (unsigned long)snapshot.reinit_recovery_count,
              (unsigned long)snapshot.spi_abort_count,
              (unsigned long)snapshot.invalid_frame_count,
              (unsigned long)snapshot.zero_frame_count,
              (unsigned long)snapshot.last_frame_age_ms,
              radar_state_name(snapshot.state),
              (unsigned long)snapshot.task_stack_high_water_words);
#endif
}

void radar_stop(void)
{
    if (latest_result.running)
    {
        (void)xensiv_bgt60trxx_start_frame(&radar_device.dev, false);
        latest_result.running = false;
        set_radar_state(RADAR_STATE_STOPPED);
        publish_presence_distance(false, false, 0.0f);
        RADAR_LOG("stopped");
    }
}
