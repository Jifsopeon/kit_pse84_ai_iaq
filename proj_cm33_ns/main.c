/*******************************************************************************
* File Name        : main.c
*
* Description      : CM33 non-secure application with SEN66 continuous streaming.
*******************************************************************************/

#include "cybsp.h"
#include "cy_time.h"
#include "cy_retarget_io.h"
#include "cy_syspm_ppu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"

#include "sen66_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "app_ble_iaq.h"
#include "app_radar.h"
#include "sen66_prediction.h"
#include "shared_mem.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CM55_BOOT_WAIT_TIME_USEC            (10U)
#define CM55_APP_BOOT_ADDR                  (CY_CM55_APP_BOOT_ADDR)
#define LPTIMER_0_WAIT_TIME_USEC            (62U)
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

#define DEBUG_UART_BAUDRATE                 (115200U)
#define SEN66_TASK_PRIORITY                 (configMAX_PRIORITIES - 1U)
#define SEN66_TASK_STACK_SIZE               (2048U)
#define RADAR_TASK_PRIORITY                 (tskIDLE_PRIORITY + 1U)
#define RADAR_TASK_STACK_SIZE               (4096U)
#define RADAR_TASK_PERIOD_MS                (20U)
#define CM55_EVENT_TASK_PRIORITY            (tskIDLE_PRIORITY + 1U)
#define CM55_EVENT_TASK_STACK_SIZE          (1024U)
#define CM55_EVENT_POLL_PERIOD_MS           (50U)
#define CM55_START_SETTLE_DELAY_US          (100U)
#define SEN66_STARTUP_DELAY_MS              (1200U)
#define SEN66_SAMPLE_PERIOD_DEFAULT_MS      (1000U)
#define SEN66_SAMPLE_PERIOD_MIN_MS          (1000U)
#define SEN66_SAMPLE_PERIOD_MAX_MS          (60000U)
#define SW1_POLL_PERIOD_MS                  (20U)
#define SW1_DEBOUNCE_POLLS                  (3U)
#define MANUAL_LABEL_NO_SMOKING             "no smoking"
#define MANUAL_LABEL_SMOKING                "smoking"
#define APP_DISTANCE_MAX_CM                 (500U)
#define APP_SERIAL_LINE_SIZE                (384U)
#define APP_BLE_ENVIRONMENTAL_DATA_LEN      (20U)
#define APP_BLE_DISTANCE_OFFSET             (APP_BLE_ENVIRONMENTAL_DATA_LEN)
#define APP_BLE_MANUAL_LABEL_OFFSET         (22U)
#define APP_BLE_PREDICTED_OFFSET            (23U)
#define APP_BLE_MANUAL_LABEL_NO_SMOKING     (0U)
#define APP_BLE_MANUAL_LABEL_SMOKING        (1U)
#define APP_SEN66_TUNING_VERIFY_FAILED      (-1)
#define APP_SEN66_VOC_INDEX_OFFSET          (1)
#define APP_SEN66_VOC_LEARNING_TIME_OFFSET_H (720)
#define APP_SEN66_VOC_LEARNING_TIME_GAIN_H  (12)
#define APP_SEN66_VOC_GATING_MAX_DURATION_MIN (180)
#define APP_SEN66_VOC_STD_INITIAL           (50)
#define APP_SEN66_VOC_GAIN_FACTOR           (1000)
#define APP_SEN66_NOX_INDEX_OFFSET          (1)
#define APP_SEN66_NOX_LEARNING_TIME_OFFSET_H (720)
#define APP_SEN66_NOX_LEARNING_TIME_GAIN_H  (12)
#define APP_SEN66_NOX_GATING_MAX_DURATION_MIN (720)
#define APP_SEN66_NOX_STD_INITIAL           (50)
#define APP_SEN66_NOX_GAIN_FACTOR           (1000)

_Static_assert((APP_SEN66_VOC_INDEX_OFFSET >= 1) && (APP_SEN66_VOC_INDEX_OFFSET <= 250),
               "VOC index offset is outside the SEN66 supported range");
_Static_assert((APP_SEN66_VOC_LEARNING_TIME_OFFSET_H >= 1) && (APP_SEN66_VOC_LEARNING_TIME_OFFSET_H <= 1000),
               "VOC learning time offset is outside the SEN66 supported range");
_Static_assert((APP_SEN66_VOC_LEARNING_TIME_GAIN_H >= 1) && (APP_SEN66_VOC_LEARNING_TIME_GAIN_H <= 1000),
               "VOC learning time gain is outside the SEN66 supported range");
_Static_assert((APP_SEN66_VOC_GATING_MAX_DURATION_MIN >= 0) && (APP_SEN66_VOC_GATING_MAX_DURATION_MIN <= 3000),
               "VOC gating max duration is outside the SEN66 supported range");
_Static_assert((APP_SEN66_VOC_STD_INITIAL >= 10) && (APP_SEN66_VOC_STD_INITIAL <= 5000),
               "VOC std initial is outside the SEN66 supported range");
_Static_assert((APP_SEN66_VOC_GAIN_FACTOR >= 1) && (APP_SEN66_VOC_GAIN_FACTOR <= 1000),
               "VOC gain factor is outside the SEN66 supported range");
_Static_assert((APP_SEN66_NOX_INDEX_OFFSET >= 1) && (APP_SEN66_NOX_INDEX_OFFSET <= 250),
               "NOx index offset is outside the SEN66 supported range");
_Static_assert((APP_SEN66_NOX_LEARNING_TIME_OFFSET_H >= 1) && (APP_SEN66_NOX_LEARNING_TIME_OFFSET_H <= 1000),
               "NOx learning time offset is outside the SEN66 supported range");
_Static_assert(APP_SEN66_NOX_LEARNING_TIME_GAIN_H == 12,
               "NOx learning time gain must be set to 12 hours");
_Static_assert((APP_SEN66_NOX_GATING_MAX_DURATION_MIN >= 0) && (APP_SEN66_NOX_GATING_MAX_DURATION_MIN <= 3000),
               "NOx gating max duration is outside the SEN66 supported range");
_Static_assert(APP_SEN66_NOX_STD_INITIAL == 50,
               "NOx std initial must be set to 50");
_Static_assert((APP_SEN66_NOX_GAIN_FACTOR >= 1) && (APP_SEN66_NOX_GAIN_FACTOR <= 1000),
               "NOx gain factor is outside the SEN66 supported range");

#if (APP_BLE_DISTANCE_OFFSET + 2U) != APP_BLE_MANUAL_LABEL_OFFSET
#error "BLE radar distance field must end immediately before the manual label byte"
#endif

#if (APP_BLE_MANUAL_LABEL_OFFSET + 1U) != APP_BLE_PREDICTED_OFFSET
#error "BLE predicted byte must immediately follow the manual label byte"
#endif

#if (APP_BLE_PREDICTED_OFFSET + 1U) != APP_BLE_IAQ_PACKET_LEN
#error "BLE predicted byte must be the final IAQ packet field"
#endif

#if defined(CYBSP_CM33_LPTIMER_0)
static mtb_hal_lptimer_t lptimer_obj;
#endif

#if defined(CYBSP_RTC)
static mtb_hal_rtc_t rtc_obj;
#endif

static cy_stc_scb_uart_context_t debug_uart_context;
static mtb_hal_uart_t debug_uart_obj;

static volatile bool app_manual_smoking = false;
static volatile uint16_t app_distance_cm = 0U;

static const char *app_event_to_string(app_event_t event)
{
    switch (event)
    {
        case APP_EVENT_CAMERA_INITIALIZED:
            return "Camera initialized";
        case APP_EVENT_CAMERA_FAILED:
            return "Camera failed";
        case APP_EVENT_DISPLAY_INITIALIZED:
            return "Display initialized";
        case APP_EVENT_DISPLAY_FAILED:
            return "Display failed";
        case APP_EVENT_MODEL_INITIALIZED:
            return "Model initialized";
        case APP_EVENT_MODEL_FAILED:
            return "Model failed";
        case APP_EVENT_INFERENCE_FAILED:
            return "Inference failed";
        case APP_EVENT_SMOKE_DETECTED:
            return "Smoke";
        case APP_EVENT_CM55_BOOT_ENTERED:
            return "[CM55] Boot";
        case APP_EVENT_CM55_MAIN_ENTERED:
            return "[CM55] Main";
        case APP_EVENT_CM55_RTOS_STARTED:
            return "[CM55] RTOS";
        case APP_EVENT_CM55_APP_INIT_ENTERED:
            return "[CM55] App init";
        case APP_EVENT_CM55_DISPLAY_INIT_ENTERED:
            return "[CM55] Display init";
        case APP_EVENT_CM55_DISPLAY_INIT_DONE:
            return "[CM55] Display init done";
        case APP_EVENT_CM55_USB_INIT_ENTERED:
            return "[CM55] USB init";
        case APP_EVENT_CM55_USB_INIT_DONE:
            return "[CM55] USB init done";
        case APP_EVENT_CM55_CAMERA_INIT_ENTERED:
            return "[CM55] Camera init";
        case APP_EVENT_CM55_CAMERA_INIT_DONE:
            return "[CM55] Camera init done";
        case APP_EVENT_CM55_MODEL_INIT_ENTERED:
            return "[CM55] Model init";
        case APP_EVENT_CM55_MODEL_INIT_DONE:
            return "[CM55] Model init done";
        case APP_EVENT_CM55_VIDEO_TASK_STARTED:
            return "[CM55] Video task";
        default:
            return NULL;
    }
}

static bool distance_m_is_valid(float distance_m)
{
    return isfinite(distance_m) && (distance_m > 0.0f);
}

static void put_u16_le(uint8_t *buffer, uint16_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t)(value & 0xFFU);
    buffer[offset + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void put_i16_le(uint8_t *buffer, uint16_t offset, int16_t value)
{
    put_u16_le(buffer, offset, (uint16_t)value);
}

static void handle_app_error(void)
{
    __disable_irq();
    CY_ASSERT(0);
    while (true)
    {
    }
}

static void setup_retarget_io(void)
{
    cy_rslt_t result;

    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW,
                                         &CYBSP_DEBUG_UART_config,
                                         &debug_uart_context);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&debug_uart_obj,
                                &CYBSP_DEBUG_UART_hal_config,
                                &debug_uart_context,
                                NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    result = mtb_hal_uart_set_baud(&debug_uart_obj, DEBUG_UART_BAUDRATE, NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    result = cy_retarget_io_init(&debug_uart_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }
}

#if defined(CYBSP_RTC)
static void setup_clib_support(void)
{
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);
    mtb_clib_support_init(&rtc_obj);
}
#endif

#if defined(CYBSP_CM33_LPTIMER_0)
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

static void setup_tickless_idle_timer(void)
{
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    cy_en_sysint_status_t interrupt_init_status = Cy_SysInt_Init(&lptimer_intr_cfg,
                                                                 lptimer_interrupt_handler);
    if (CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    cy_en_mcwdt_status_t mcwdt_init_status = Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW,
                                                           &CYBSP_CM33_LPTIMER_0_config);
    if (CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }

    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW, CY_MCWDT_CTR_Msk, LPTIMER_0_WAIT_TIME_USEC);

    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj, &CYBSP_CM33_LPTIMER_0_hal_config);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    cyabs_rtos_set_lptimer(&lptimer_obj);
}
#endif

static void format_distance_slot_m(float distance_m, char* buffer, size_t buffer_size)
{
    if ((NULL == buffer) || (0U == buffer_size))
    {
        return;
    }

    if (!distance_m_is_valid(distance_m))
    {
        snprintf(buffer, buffer_size, "0");
        return;
    }

    uint32_t distance_hundredths_m = (uint32_t)((distance_m * 100.0f) + 0.5f);
    snprintf(buffer, buffer_size, "%lu.%02lu",
             (unsigned long)(distance_hundredths_m / 100U),
             (unsigned long)(distance_hundredths_m % 100U));
}

static uint16_t radar_distance_m_to_mm(float distance_m)
{
    if (!distance_m_is_valid(distance_m))
    {
        return 0U;
    }

    float distance_mm = distance_m * 1000.0f;
    if (distance_mm >= (float)UINT16_MAX)
    {
        return UINT16_MAX;
    }

    return (uint16_t)(distance_mm + 0.5f);
}

static void put_radar_distance_le(uint8_t *buffer, float distance_m)
{
    /* Radar distance field:
     *
     * Bytes 20-21: selected presence distance, uint16_t millimetres, little-endian.
     *
     * A value of 0 means that no valid presence distance exists.
     * Values are displayed to users in metres.
     */
    put_u16_le(buffer, APP_BLE_DISTANCE_OFFSET, radar_distance_m_to_mm(distance_m));
}

static void format_u16_scaled_1(uint16_t value, char* buffer, size_t buffer_size)
{
    if (0xFFFFU == value)
    {
        snprintf(buffer, buffer_size, "NA");
    }
    else
    {
        snprintf(buffer, buffer_size, "%u.%u", value / 10U, value % 10U);
    }
}

static void format_i16_scaled_1(int16_t value, char* buffer, size_t buffer_size)
{
    if (0x7FFF == value)
    {
        snprintf(buffer, buffer_size, "NA");
    }
    else
    {
        int16_t whole = value / 10;
        int16_t fraction = value % 10;

        if (fraction < 0)
        {
            fraction = (int16_t)-fraction;
        }

        snprintf(buffer, buffer_size, "%d.%d", whole, fraction);
    }
}

static void format_i16_scaled_2(int16_t value, int16_t scale, char* buffer, size_t buffer_size)
{
    if (0x7FFF == value)
    {
        snprintf(buffer, buffer_size, "NA");
    }
    else
    {
        int32_t scaled_value = ((int32_t)value * 100) / scale;
        int32_t whole = scaled_value / 100;
        int32_t fraction = scaled_value % 100;

        if (fraction < 0)
        {
            fraction = -fraction;
        }

        snprintf(buffer, buffer_size, "%ld.%02ld", (long)whole, (long)fraction);
    }
}

static bool sen66_tuning_matches(int16_t actual_index_offset,
                                 int16_t actual_learning_time_offset_hours,
                                 int16_t actual_learning_time_gain_hours,
                                 int16_t actual_gating_max_duration_minutes,
                                 int16_t actual_std_initial,
                                 int16_t actual_gain_factor,
                                 int16_t expected_index_offset,
                                 int16_t expected_learning_time_offset_hours,
                                 int16_t expected_learning_time_gain_hours,
                                 int16_t expected_gating_max_duration_minutes,
                                 int16_t expected_std_initial,
                                 int16_t expected_gain_factor)
{
    return (actual_index_offset == expected_index_offset) &&
           (actual_learning_time_offset_hours == expected_learning_time_offset_hours) &&
           (actual_learning_time_gain_hours == expected_learning_time_gain_hours) &&
           (actual_gating_max_duration_minutes == expected_gating_max_duration_minutes) &&
           (actual_std_initial == expected_std_initial) &&
           (actual_gain_factor == expected_gain_factor);
}

static int16_t app_sen66_apply_algorithm_tuning(void)
{
    int16_t index_offset;
    int16_t learning_time_offset_hours;
    int16_t learning_time_gain_hours;
    int16_t gating_max_duration_minutes;
    int16_t std_initial;
    int16_t gain_factor;
    int16_t error;

    printf("[SEN66] applying algorithm tuning\r\n");

    error = sen66_stop_measurement();
    if (NO_ERROR != error)
    {
        printf("[SEN66] tuning idle request failed status=%d\r\n", error);
        return error;
    }
    printf("[SEN66] tuning sensor idle confirmed via stop_measurement\r\n");

    printf("[SEN66] applying VOC tuning index_offset=%d learning_offset_h=%d learning_gain_h=%d gating_min=%d std_initial=%d gain=%d\r\n",
           APP_SEN66_VOC_INDEX_OFFSET,
           APP_SEN66_VOC_LEARNING_TIME_OFFSET_H,
           APP_SEN66_VOC_LEARNING_TIME_GAIN_H,
           APP_SEN66_VOC_GATING_MAX_DURATION_MIN,
           APP_SEN66_VOC_STD_INITIAL,
           APP_SEN66_VOC_GAIN_FACTOR);

    error = sen66_set_voc_algorithm_tuning_parameters(APP_SEN66_VOC_INDEX_OFFSET,
                                                      APP_SEN66_VOC_LEARNING_TIME_OFFSET_H,
                                                      APP_SEN66_VOC_LEARNING_TIME_GAIN_H,
                                                      APP_SEN66_VOC_GATING_MAX_DURATION_MIN,
                                                      APP_SEN66_VOC_STD_INITIAL,
                                                      APP_SEN66_VOC_GAIN_FACTOR);
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning failed status=%d\r\n", error);
        return error;
    }

    error = sen66_get_voc_algorithm_tuning_parameters(&index_offset,
                                                      &learning_time_offset_hours,
                                                      &learning_time_gain_hours,
                                                      &gating_max_duration_minutes,
                                                      &std_initial,
                                                      &gain_factor);
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning readback failed status=%d\r\n", error);
        return error;
    }

    printf("[SEN66] VOC tuning readback index_offset=%d learning_offset_h=%d learning_gain_h=%d gating_min=%d std_initial=%d gain=%d\r\n",
           index_offset,
           learning_time_offset_hours,
           learning_time_gain_hours,
           gating_max_duration_minutes,
           std_initial,
           gain_factor);

    if (!sen66_tuning_matches(index_offset,
                              learning_time_offset_hours,
                              learning_time_gain_hours,
                              gating_max_duration_minutes,
                              std_initial,
                              gain_factor,
                              APP_SEN66_VOC_INDEX_OFFSET,
                              APP_SEN66_VOC_LEARNING_TIME_OFFSET_H,
                              APP_SEN66_VOC_LEARNING_TIME_GAIN_H,
                              APP_SEN66_VOC_GATING_MAX_DURATION_MIN,
                              APP_SEN66_VOC_STD_INITIAL,
                              APP_SEN66_VOC_GAIN_FACTOR))
    {
        printf("[SEN66] VOC tuning verify failed\r\n");
        return APP_SEN66_TUNING_VERIFY_FAILED;
    }

    printf("[SEN66] VOC tuning applied\r\n");

    printf("[SEN66] applying NOx tuning index_offset=%d learning_offset_h=%d learning_gain_h=%d gating_min=%d std_initial=%d gain=%d\r\n",
           APP_SEN66_NOX_INDEX_OFFSET,
           APP_SEN66_NOX_LEARNING_TIME_OFFSET_H,
           APP_SEN66_NOX_LEARNING_TIME_GAIN_H,
           APP_SEN66_NOX_GATING_MAX_DURATION_MIN,
           APP_SEN66_NOX_STD_INITIAL,
           APP_SEN66_NOX_GAIN_FACTOR);

    error = sen66_set_nox_algorithm_tuning_parameters(APP_SEN66_NOX_INDEX_OFFSET,
                                                      APP_SEN66_NOX_LEARNING_TIME_OFFSET_H,
                                                      APP_SEN66_NOX_LEARNING_TIME_GAIN_H,
                                                      APP_SEN66_NOX_GATING_MAX_DURATION_MIN,
                                                      APP_SEN66_NOX_STD_INITIAL,
                                                      APP_SEN66_NOX_GAIN_FACTOR);
    if (NO_ERROR != error)
    {
        printf("[SEN66] NOx tuning failed status=%d\r\n", error);
        return error;
    }

    error = sen66_get_nox_algorithm_tuning_parameters(&index_offset,
                                                      &learning_time_offset_hours,
                                                      &learning_time_gain_hours,
                                                      &gating_max_duration_minutes,
                                                      &std_initial,
                                                      &gain_factor);
    if (NO_ERROR != error)
    {
        printf("[SEN66] NOx tuning readback failed status=%d\r\n", error);
        return error;
    }

    printf("[SEN66] NOx tuning readback index_offset=%d learning_offset_h=%d learning_gain_h=%d gating_min=%d std_initial=%d gain=%d\r\n",
           index_offset,
           learning_time_offset_hours,
           learning_time_gain_hours,
           gating_max_duration_minutes,
           std_initial,
           gain_factor);

    if (!sen66_tuning_matches(index_offset,
                              learning_time_offset_hours,
                              learning_time_gain_hours,
                              gating_max_duration_minutes,
                              std_initial,
                              gain_factor,
                              APP_SEN66_NOX_INDEX_OFFSET,
                              APP_SEN66_NOX_LEARNING_TIME_OFFSET_H,
                              APP_SEN66_NOX_LEARNING_TIME_GAIN_H,
                              APP_SEN66_NOX_GATING_MAX_DURATION_MIN,
                              APP_SEN66_NOX_STD_INITIAL,
                              APP_SEN66_NOX_GAIN_FACTOR))
    {
        printf("[SEN66] NOx tuning verify failed\r\n");
        return APP_SEN66_TUNING_VERIFY_FAILED;
    }

    printf("[SEN66] NOx tuning applied\r\n");
    printf("[SEN66] algorithm tuning configuration complete\r\n");
    return NO_ERROR;
}

void set_manual_label(bool manual_smoking)
{
    bool previous_manual_smoking = app_manual_smoking;
    app_manual_smoking = manual_smoking;
    Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN,
                  manual_smoking ? CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF);
    if (previous_manual_smoking != manual_smoking)
    {
        printf("[LABEL] value=%s\r\n",
               manual_smoking ? MANUAL_LABEL_SMOKING : MANUAL_LABEL_NO_SMOKING);
    }
}

void set_distance_cm(uint16_t distance_cm)
{
    app_distance_cm = (distance_cm > APP_DISTANCE_MAX_CM) ? APP_DISTANCE_MAX_CM : distance_cm;
}

static bool sw1_is_pressed(void)
{
    return (0U == Cy_GPIO_Read(CYBSP_SW1_PORT, CYBSP_SW1_PIN));
}

static void update_manual_label(bool* manual_smoking, bool* sw1_was_pressed,
                                uint8_t* sw1_stable_pressed_count)
{
    bool sw1_pressed = sw1_is_pressed();

    if (sw1_pressed)
    {
        if (*sw1_stable_pressed_count < SW1_DEBOUNCE_POLLS)
        {
            (*sw1_stable_pressed_count)++;
        }

        if ((*sw1_stable_pressed_count >= SW1_DEBOUNCE_POLLS) && !(*sw1_was_pressed))
        {
            *manual_smoking = !(*manual_smoking);
            set_manual_label(*manual_smoking);
            *sw1_was_pressed = true;
        }
    }
    else
    {
        *sw1_stable_pressed_count = 0U;
        *sw1_was_pressed = false;
    }
}

static bool drain_ble_commands(bool* manual_smoking, uint32_t* sample_period_ms)
{
    app_ble_iaq_command_t command;
    bool immediate_sample = false;

    while (app_ble_iaq_receive_command(&command, 0U))
    {
        switch (command.id)
        {
            case APP_BLE_IAQ_CMD_START_STREAMING:
                app_ble_iaq_set_streaming_enabled(true);
                break;

            case APP_BLE_IAQ_CMD_STOP_STREAMING:
                app_ble_iaq_set_streaming_enabled(false);
                break;

            case APP_BLE_IAQ_CMD_SET_INTERVAL_MS:
                *sample_period_ms = (uint32_t)command.value_u16;
                if (*sample_period_ms < SEN66_SAMPLE_PERIOD_MIN_MS)
                {
                    *sample_period_ms = SEN66_SAMPLE_PERIOD_MIN_MS;
                }
                if (*sample_period_ms > SEN66_SAMPLE_PERIOD_MAX_MS)
                {
                    *sample_period_ms = SEN66_SAMPLE_PERIOD_MAX_MS;
                }
                printf("[BLE] sample interval:%lu ms\r\n", (unsigned long)*sample_period_ms);
                break;

            case APP_BLE_IAQ_CMD_IMMEDIATE_SAMPLE:
                immediate_sample = true;
                break;

            case APP_BLE_IAQ_CMD_SET_DISTANCE_CM:
                set_distance_cm(command.value_u16);
                break;

            case APP_BLE_IAQ_CMD_SET_MANUAL_LABEL:
                *manual_smoking = command.value_bool;
                set_manual_label(*manual_smoking);
                break;

            default:
                printf("[BLE] unknown command:0x%02X\r\n", (unsigned int)command.id);
                break;
        }
    }

    return immediate_sample;
}

static void wait_sample_period_and_update_label(bool* manual_smoking, bool* sw1_was_pressed,
                                                uint8_t* sw1_stable_pressed_count,
                                                uint32_t* sample_period_ms)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < *sample_period_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(SW1_POLL_PERIOD_MS));
        update_manual_label(manual_smoking, sw1_was_pressed, sw1_stable_pressed_count);
        app_ble_iaq_process();
        radar_log_watchdog();
        if (drain_ble_commands(manual_smoking, sample_period_ms))
        {
            break;
        }
        elapsed_ms += SW1_POLL_PERIOD_MS;
    }
}

static void task_sen66_stream(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    int16_t error;
    int8_t serial_number[32] = {0};
    bool manual_smoking = false;
    bool sw1_was_pressed = sw1_is_pressed();
    uint8_t sw1_stable_pressed_count = 0U;
    uint32_t sample_period_ms = SEN66_SAMPLE_PERIOD_DEFAULT_MS;
    uint16_t sequence = 0U;
    bool sensor_streaming = true;

    set_manual_label(manual_smoking);
    set_distance_cm(0U);
    sen66_prediction_init();

    sensirion_i2c_hal_init();
    sen66_init(SEN66_I2C_ADDR_6B);

    error = sen66_device_reset();
    if (NO_ERROR != error)
    {
        printf("sen66_device_reset_error:%d\r\n", error);
    }

    vTaskDelay(pdMS_TO_TICKS(SEN66_STARTUP_DELAY_MS));

    error = sen66_get_serial_number(serial_number, sizeof(serial_number));
    if (NO_ERROR == error)
    {
        printf("sen66_serial:%s\r\n", (char*)serial_number);
    }
    else
    {
        printf("sen66_serial_error:%d\r\n", error);
    }

    error = app_sen66_apply_algorithm_tuning();
    if (NO_ERROR != error)
    {
        printf("[SEN66] algorithm tuning failed status=%d; continuing without verified tuning\r\n", error);
    }

    error = sen66_start_continuous_measurement();
    if (NO_ERROR != error)
    {
        printf("sen66_start_error:%d\r\n", error);
        sensor_streaming = false;
    }

    wait_sample_period_and_update_label(&manual_smoking, &sw1_was_pressed,
                                        &sw1_stable_pressed_count, &sample_period_ms);

    for (;;)
    {
        uint16_t pm1p0 = 0U;
        uint16_t pm2p5 = 0U;
        uint16_t pm4p0 = 0U;
        uint16_t pm10p0 = 0U;
        int16_t humidity = 0;
        int16_t temperature = 0;
        int16_t voc = 0;
        int16_t nox = 0;
        uint16_t co2 = 0U;
        char pm1p0_text[16] = {0};
        char pm2p5_text[16] = {0};
        char pm4p0_text[16] = {0};
        char pm10p0_text[16] = {0};
        char humidity_text[16] = {0};
        char temperature_text[16] = {0};
        char voc_text[16] = {0};
        char nox_text[16] = {0};
        char distance_text[16] = {0};
        char serial_line[APP_SERIAL_LINE_SIZE] = {0};
        uint8_t ble_packet[APP_BLE_IAQ_PACKET_LEN] = {0};
        uint8_t predicted = SEN66_PREDICTION_NOT_READY;

        wait_sample_period_and_update_label(&manual_smoking, &sw1_was_pressed,
                                            &sw1_stable_pressed_count, &sample_period_ms);

        if (!sensor_streaming)
        {
            continue;
        }

        error = sen66_read_measured_values_as_integers(&pm1p0, &pm2p5, &pm4p0, &pm10p0,
                                                       &humidity, &temperature, &voc, &nox, &co2);
        if (NO_ERROR != error)
        {
            printf("sen66_read_error:%d\r\n", error);
            continue;
        }

        radar_result_t radar_result;
        radar_get_latest_result(&radar_result);
        float radar_distance_m = (radar_result.presence_detected &&
                                  radar_result.distance_valid) ?
                                 radar_result.distance_m : 0.0f;

        manual_smoking = app_manual_smoking;
        format_u16_scaled_1(pm1p0, pm1p0_text, sizeof(pm1p0_text));
        format_u16_scaled_1(pm2p5, pm2p5_text, sizeof(pm2p5_text));
        format_u16_scaled_1(pm4p0, pm4p0_text, sizeof(pm4p0_text));
        format_u16_scaled_1(pm10p0, pm10p0_text, sizeof(pm10p0_text));
        format_i16_scaled_2(humidity, 100, humidity_text, sizeof(humidity_text));
        format_i16_scaled_2(temperature, 200, temperature_text, sizeof(temperature_text));
        format_i16_scaled_1(voc, voc_text, sizeof(voc_text));
        format_i16_scaled_1(nox, nox_text, sizeof(nox_text));
        format_distance_slot_m(radar_distance_m, distance_text, sizeof(distance_text));
        (void)sen66_prediction_update(pm1p0, pm2p5, pm4p0, pm10p0,
                                      voc, nox, co2, &predicted);

        snprintf(serial_line, sizeof(serial_line),
                 "PM1.0: %s  PM2.5: %s  PM4.0: %s  PM10.0: %s  Humidity: %s  Temperature: %s  VOC: %s  NOx: %s  CO2: %u  Distance: %s  Manual Label: %s, Predicted: %s\r\n",
                 pm1p0_text, pm2p5_text, pm4p0_text, pm10p0_text,
                 humidity_text, temperature_text, voc_text, nox_text, co2, distance_text,
                 manual_smoking ? MANUAL_LABEL_SMOKING : MANUAL_LABEL_NO_SMOKING,
                 sen66_prediction_format(predicted));
        printf("%s", serial_line);

        put_u16_le(ble_packet, 0U, sequence++);
        put_u16_le(ble_packet, 2U, pm1p0);
        put_u16_le(ble_packet, 4U, pm2p5);
        put_u16_le(ble_packet, 6U, pm4p0);
        put_u16_le(ble_packet, 8U, pm10p0);
        put_u16_le(ble_packet, 10U, (uint16_t)humidity);
        put_i16_le(ble_packet, 12U, temperature);
        put_u16_le(ble_packet, 14U, (uint16_t)nox);
        put_u16_le(ble_packet, 16U, (uint16_t)voc);
        put_u16_le(ble_packet, 18U, co2);
        put_radar_distance_le(ble_packet, radar_distance_m);
        ble_packet[APP_BLE_MANUAL_LABEL_OFFSET] = manual_smoking ?
                                                  APP_BLE_MANUAL_LABEL_SMOKING :
                                                  APP_BLE_MANUAL_LABEL_NO_SMOKING;
        ble_packet[APP_BLE_PREDICTED_OFFSET] = predicted;
        app_ble_iaq_notify_sensor_packet(ble_packet);
    }
}

static void task_radar(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    cy_rslt_t result = radar_init();
    if (CY_RSLT_SUCCESS == result)
    {
        result = radar_start();
    }

    if (CY_RSLT_SUCCESS != result)
    {
        printf("Radar failed\r\n");
        vTaskSuspend(NULL);
    }

    printf("Radar initialized\r\n");

    for (;;)
    {
        cy_rslt_t process_result = radar_process();
        if (CY_RSLT_SUCCESS != process_result)
        {
            if (CY_RSLT_SUCCESS != radar_start())
            {
                vTaskDelay(pdMS_TO_TICKS(1000U));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(RADAR_TASK_PERIOD_MS));
    }
}

static uint32_t cm55_ppu_power_mode(const void *base)
{
    return cy_pd_ppu_get_power_mode((struct ppu_v1_reg *)base);
}

static void print_cm55_start_status(const char *phase)
{
    volatile const uint32_t *vector_table = (volatile const uint32_t *)CM55_APP_BOOT_ADDR;
    uint32_t msp = vector_table[0];
    uint32_t reset = vector_table[1];
    uint32_t core_status = Cy_SysGetCM55Status(MXCM55);

    printf("[CM55 START] %s vector=0x%08lx msp=0x%08lx reset=0x%08lx core=%lu pd1=%lu appcpuss=%lu appcpu=%lu socmem=%lu shared=0x%08lx end=0x%08lx size=%lu api=void\r\n",
           phase,
           (unsigned long)CM55_APP_BOOT_ADDR,
           (unsigned long)msp,
           (unsigned long)reset,
           (unsigned long)core_status,
           (unsigned long)cm55_ppu_power_mode(PWRMODE_PPU_PD1),
           (unsigned long)cm55_ppu_power_mode(PPU_APPCPUSS),
           (unsigned long)cm55_ppu_power_mode(PPU_CM55_0),
           (unsigned long)cm55_ppu_power_mode(SOCMEM_PPU_SOCMEM),
           (unsigned long)app_shared_data_address(),
           (unsigned long)app_shared_data_end_address(),
           (unsigned long)app_shared_data_size());
}

static const char *cm55_boot_state_to_string(uint32_t state)
{
    switch (state)
    {
        case APP_CM55_BOOT_NONE: return "NONE";
        case APP_CM55_BOOT_RESET_HANDLER: return "RESET_HANDLER";
        case APP_CM55_BOOT_SYSTEM_INIT_ENTER: return "SYSTEM_INIT_ENTER";
        case APP_CM55_BOOT_SYSTEM_INIT_DONE: return "SYSTEM_INIT_DONE";
        case APP_CM55_BOOT_CRT_INIT_ENTER: return "CRT_INIT_ENTER";
        case APP_CM55_BOOT_MAIN_ENTER: return "MAIN_ENTER";
        case APP_CM55_BOOT_PLATFORM_INIT_ENTER: return "PLATFORM_INIT_ENTER";
        case APP_CM55_BOOT_PLATFORM_INIT_DONE: return "PLATFORM_INIT_DONE";
        case APP_CM55_BOOT_PSRAM_INIT_ENTER: return "PSRAM_INIT_ENTER";
        case APP_CM55_BOOT_PSRAM_INIT_DONE: return "PSRAM_INIT_DONE";
        case APP_CM55_BOOT_RTOS_CREATE: return "RTOS_CREATE";
        case APP_CM55_BOOT_SCHEDULER_STARTED: return "SCHEDULER_STARTED";
        case APP_CM55_BOOT_APP_INIT_ENTER: return "APP_INIT_ENTER";
        case APP_CM55_BOOT_DISPLAY_INIT_ENTER: return "DISPLAY_INIT_ENTER";
        case APP_CM55_BOOT_USB_INIT_ENTER: return "USB_INIT_ENTER";
        case APP_CM55_BOOT_MODEL_INIT_ENTER: return "MODEL_INIT_ENTER";
        case APP_CM55_BOOT_DISPLAY_INIT_DONE: return "DISPLAY_INIT_DONE";
        case APP_CM55_BOOT_USB_INIT_DONE: return "USB_INIT_DONE";
        case APP_CM55_BOOT_MODEL_INIT_DONE: return "MODEL_INIT_DONE";
        case APP_CM55_BOOT_CAMERA_INIT_ENTER: return "CAMERA_INIT_ENTER";
        case APP_CM55_BOOT_CAMERA_INIT_DONE: return "CAMERA_INIT_DONE";
        case APP_CM55_BOOT_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *cm55_fault_to_string(uint32_t fault)
{
    switch (fault)
    {
        case APP_CM55_FAULT_HARDFAULT: return "HardFault";
        case APP_CM55_FAULT_MEMMANAGE: return "MemManage";
        case APP_CM55_FAULT_BUSFAULT: return "BusFault";
        case APP_CM55_FAULT_USAGEFAULT: return "UsageFault";
        case APP_CM55_FAULT_DEBUGMON: return "DebugMon";
        default: return "Unknown";
    }
}

static void print_cfsr_bits(uint32_t cfsr)
{
    printf("[CM55 FAULT] MMFSR:");
    if (0U == (cfsr & 0x000000FFUL)) { printf(" none"); }
    if (0U != (cfsr & (1UL << 0))) { printf(" IACCVIOL"); }
    if (0U != (cfsr & (1UL << 1))) { printf(" DACCVIOL"); }
    if (0U != (cfsr & (1UL << 3))) { printf(" MUNSTKERR"); }
    if (0U != (cfsr & (1UL << 4))) { printf(" MSTKERR"); }
    if (0U != (cfsr & (1UL << 5))) { printf(" MLSPERR"); }
    if (0U != (cfsr & (1UL << 7))) { printf(" MMARVALID"); }
    printf("\r\n");

    printf("[CM55 FAULT] BFSR:");
    if (0U == (cfsr & 0x0000FF00UL)) { printf(" none"); }
    if (0U != (cfsr & (1UL << 8))) { printf(" IBUSERR"); }
    if (0U != (cfsr & (1UL << 9))) { printf(" PRECISERR"); }
    if (0U != (cfsr & (1UL << 10))) { printf(" IMPRECISERR"); }
    if (0U != (cfsr & (1UL << 11))) { printf(" UNSTKERR"); }
    if (0U != (cfsr & (1UL << 12))) { printf(" STKERR"); }
    if (0U != (cfsr & (1UL << 13))) { printf(" LSPERR"); }
    if (0U != (cfsr & (1UL << 15))) { printf(" BFARVALID"); }
    printf("\r\n");

    printf("[CM55 FAULT] UFSR:");
    if (0U == (cfsr & 0xFFFF0000UL)) { printf(" none"); }
    if (0U != (cfsr & (1UL << 16))) { printf(" UNDEFINSTR"); }
    if (0U != (cfsr & (1UL << 17))) { printf(" INVSTATE"); }
    if (0U != (cfsr & (1UL << 18))) { printf(" INVPC"); }
    if (0U != (cfsr & (1UL << 19))) { printf(" NOCP"); }
    if (0U != (cfsr & (1UL << 20))) { printf(" STKOF"); }
    if (0U != (cfsr & (1UL << 24))) { printf(" UNALIGNED"); }
    if (0U != (cfsr & (1UL << 25))) { printf(" DIVBYZERO"); }
    printf("\r\n");
}

static void print_hfsr_bits(uint32_t hfsr)
{
    printf("[CM55 FAULT] HFSR bits:");
    if (0U == hfsr) { printf(" none"); }
    if (0U != (hfsr & (1UL << 1))) { printf(" VECTTBL"); }
    if (0U != (hfsr & (1UL << 30))) { printf(" FORCED"); }
    if (0U != (hfsr & (1UL << 31))) { printf(" DEBUGEVT"); }
    printf("\r\n");
}

static void print_cm55_fault_report(const app_cm55_boot_state_t *state)
{
    printf("[CM55 TEST] last=%s state=0x%08lx\r\n",
           cm55_boot_state_to_string(state->last_state),
           (unsigned long)state->state);
    printf("[CM55 FAULT] type=%s fault=0x%08lx\r\n",
           cm55_fault_to_string(state->fault),
           (unsigned long)state->fault);
    printf("[CM55 FAULT] PC=0x%08lx LR=0x%08lx xPSR=0x%08lx EXC_RETURN=0x%08lx\r\n",
           (unsigned long)state->pc,
           (unsigned long)state->lr,
           (unsigned long)state->xpsr,
           (unsigned long)state->exc_return);
    printf("[CM55 FAULT] R0=0x%08lx R1=0x%08lx R2=0x%08lx R3=0x%08lx R12=0x%08lx\r\n",
           (unsigned long)state->r0,
           (unsigned long)state->r1,
           (unsigned long)state->r2,
           (unsigned long)state->r3,
           (unsigned long)state->r12);
    printf("[CM55 FAULT] MSP=0x%08lx PSP=0x%08lx MSPLIM=0x%08lx PSPLIM=0x%08lx CONTROL=0x%08lx\r\n",
           (unsigned long)state->msp,
           (unsigned long)state->psp,
           (unsigned long)state->msplim,
           (unsigned long)state->psplim,
           (unsigned long)state->control);
    printf("[CM55 FAULT] CFSR=0x%08lx HFSR=0x%08lx DFSR=0x%08lx AFSR=0x%08lx SHCSR=0x%08lx ICSR=0x%08lx VTOR=0x%08lx\r\n",
           (unsigned long)state->cfsr,
           (unsigned long)state->hfsr,
           (unsigned long)state->dfsr,
           (unsigned long)state->afsr,
           (unsigned long)state->shcsr,
           (unsigned long)state->icsr,
           (unsigned long)state->vtor);
    printf("[CM55 FAULT] BFAR=0x%08lx MMFAR=0x%08lx\r\n",
           (unsigned long)state->bfar,
           (unsigned long)state->mmfar);
    print_cfsr_bits(state->cfsr);
    print_hfsr_bits(state->hfsr);
}

static void task_cm55_events(void* arg)
{
    CY_UNUSED_PARAMETER(arg);
    app_cm55_boot_state_t previous_boot_state = {0};
    bool fault_reported = false;

#if APP_CM55_STARTUP_DIAGNOSTICS
    printf("[CM55 TEST] monitor active\r\n");
#endif

    for (;;)
    {
#if APP_CM55_STARTUP_DIAGNOSTICS
        app_cm55_boot_state_t boot_state;
        app_cm55_boot_state_read(&boot_state);
        if (boot_state.magic == APP_CM55_BOOT_STATE_MAGIC)
        {
            if (boot_state.state != previous_boot_state.state)
            {
                printf("[CM55 TEST] last=%s state=0x%08lx fault=0x%08lx\r\n",
                       cm55_boot_state_to_string(boot_state.state),
                       (unsigned long)boot_state.state,
                       (unsigned long)boot_state.fault);
            }
            if ((APP_CM55_BOOT_FAULT == boot_state.state) && !fault_reported)
            {
                print_cm55_fault_report(&boot_state);
                fault_reported = true;
            }
            if (boot_state.boot_entered != previous_boot_state.boot_entered)
            {
                printf("[CM55] boot mark:%lu\r\n", (unsigned long)boot_state.boot_entered);
            }
            if (boot_state.main_entered != previous_boot_state.main_entered)
            {
                printf("[CM55] main mark:%lu\r\n", (unsigned long)boot_state.main_entered);
            }
            if (boot_state.app_init_entered != previous_boot_state.app_init_entered)
            {
                printf("[CM55] app-init mark:%lu\r\n", (unsigned long)boot_state.app_init_entered);
            }
            if (boot_state.rtos_started != previous_boot_state.rtos_started)
            {
                printf("[CM55] rtos mark:%lu\r\n", (unsigned long)boot_state.rtos_started);
            }
            previous_boot_state = boot_state;
        }
#endif

        app_event_t event;
        while (app_event_try_receive(&event))
        {
            const char *message = app_event_to_string(event);
            if (NULL != message)
            {
                printf("%s\r\n", message);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CM55_EVENT_POLL_PERIOD_MS));
    }
}

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

#if defined(CYBSP_RTC)
    setup_clib_support();
#endif

#if defined(CYBSP_CM33_LPTIMER_0)
    setup_tickless_idle_timer();
#endif

    setup_retarget_io();
    __enable_irq();
    app_ble_iaq_init();
    app_cm55_boot_state_init();
    app_event_queue_init();

    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    Cy_SysLib_DelayUs(CM55_START_SETTLE_DELAY_US);
    print_cm55_start_status("after");

    if (pdPASS != xTaskCreate(task_cm55_events, "CM55_EVENTS", CM55_EVENT_TASK_STACK_SIZE,
                              NULL, CM55_EVENT_TASK_PRIORITY, NULL))
    {
        printf("CM55 event task failed\r\n");
    }

    if (pdPASS != xTaskCreate(task_sen66_stream, "SEN66", SEN66_TASK_STACK_SIZE,
                              NULL, SEN66_TASK_PRIORITY, NULL))
    {
        handle_app_error();
    }

    if (pdPASS != xTaskCreate(task_radar, "RADAR", RADAR_TASK_STACK_SIZE,
                              NULL, RADAR_TASK_PRIORITY, NULL))
    {
        printf("[RADAR] task create failed; continuing without radar\r\n");
    }

    vTaskStartScheduler();

    handle_app_error();
}

/* [] END OF FILE */
