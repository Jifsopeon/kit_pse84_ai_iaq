/*******************************************************************************
* File Name        : main.c
*
* Description      : CM33 non-secure application with SEN66 continuous streaming.
*******************************************************************************/

#include "cybsp.h"
#include "cy_time.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"

#include "sen66_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "app_ble_iaq.h"

#include <stdbool.h>
#include <stdio.h>

#define CM55_BOOT_WAIT_TIME_USEC            (10U)
#define CM55_APP_BOOT_ADDR                  (CYMEM_CM33_0_m55_nvm_START + CYBSP_MCUBOOT_HEADER_SIZE)
#define LPTIMER_0_WAIT_TIME_USEC            (62U)
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

#define DEBUG_UART_BAUDRATE                 (115200U)
#define SEN66_TASK_PRIORITY                 (configMAX_PRIORITIES - 1U)
#define SEN66_TASK_STACK_SIZE               (2048U)
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
#define SEN66_VOC_INDEX_OFFSET_REQUESTED    (250)
#define SEN66_VOC_GAIN_FACTOR_REQUESTED     (1000)

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

static void format_unsigned_distance_m(uint16_t distance_cm, char* buffer, size_t buffer_size)
{
    uint16_t meters = (uint16_t)(distance_cm / 100U);
    uint16_t centimeters = (uint16_t)(distance_cm % 100U);

    if (0U == distance_cm)
    {
        snprintf(buffer, buffer_size, "0");
    }
    else
    {
        snprintf(buffer, buffer_size, "%u.%02u", meters, centimeters);
    }
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

static int16_t sen66_apply_voc_tuning(void)
{
    int16_t index_offset = 100;
    int16_t learning_time_offset_hours = 12;
    int16_t learning_time_gain_hours = 12;
    int16_t gating_max_duration_minutes = 180;
    int16_t std_initial = 50;
    int16_t gain_factor = 230;
    int16_t readback_index_offset = 0;
    int16_t readback_learning_time_offset_hours = 0;
    int16_t readback_learning_time_gain_hours = 0;
    int16_t readback_gating_max_duration_minutes = 0;
    int16_t readback_std_initial = 0;
    int16_t readback_gain_factor = 0;
    int16_t error;

    printf("[SEN66] Applying VOC algorithm tuning\r\n");
    printf("[SEN66] VOC index offset requested=%d\r\n", SEN66_VOC_INDEX_OFFSET_REQUESTED);
    printf("[SEN66] VOC gain factor requested=%d\r\n", SEN66_VOC_GAIN_FACTOR_REQUESTED);

    error = sen66_stop_measurement();
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning idle request failed status=%d\r\n", error);
        return error;
    }
    printf("[SEN66] VOC tuning sensor idle confirmed via stop_measurement\r\n");

    error = sen66_get_voc_algorithm_tuning_parameters(&index_offset,
                                                      &learning_time_offset_hours,
                                                      &learning_time_gain_hours,
                                                      &gating_max_duration_minutes,
                                                      &std_initial,
                                                      &gain_factor);
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning read current failed status=%d\r\n", error);
        return error;
    }

    printf("[SEN66] VOC tuning current: index_offset=%d learning_time_offset=%d learning_time_gain=%d gating_max_duration=%d std_initial=%d gain_factor=%d\r\n",
           index_offset,
           learning_time_offset_hours,
           learning_time_gain_hours,
           gating_max_duration_minutes,
           std_initial,
           gain_factor);
    printf("[SEN66] VOC tuning preserved: learning_time_offset=%d learning_time_gain=%d gating_max_duration=%d std_initial=%d\r\n",
           learning_time_offset_hours,
           learning_time_gain_hours,
           gating_max_duration_minutes,
           std_initial);

    error = sen66_set_voc_algorithm_tuning_parameters(SEN66_VOC_INDEX_OFFSET_REQUESTED,
                                                      learning_time_offset_hours,
                                                      learning_time_gain_hours,
                                                      gating_max_duration_minutes,
                                                      std_initial,
                                                      SEN66_VOC_GAIN_FACTOR_REQUESTED);
    printf("[SEN66] VOC tuning write result=%d\r\n", error);
    if (NO_ERROR != error)
    {
        return error;
    }

    error = sen66_get_voc_algorithm_tuning_parameters(&readback_index_offset,
                                                      &readback_learning_time_offset_hours,
                                                      &readback_learning_time_gain_hours,
                                                      &readback_gating_max_duration_minutes,
                                                      &readback_std_initial,
                                                      &readback_gain_factor);
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning readback failed status=%d\r\n", error);
        return error;
    }

    printf("[SEN66] VOC tuning readback: index_offset=%d gain_factor=%d\r\n",
           readback_index_offset,
           readback_gain_factor);

    if ((SEN66_VOC_INDEX_OFFSET_REQUESTED != readback_index_offset) ||
        (SEN66_VOC_GAIN_FACTOR_REQUESTED != readback_gain_factor))
    {
        printf("[SEN66] VOC tuning verify failed index_offset=%d gain_factor=%d\r\n",
               readback_index_offset,
               readback_gain_factor);
        return -1;
    }

    printf("[SEN66] VOC tuning configuration complete\r\n");
    return NO_ERROR;
}

void set_manual_label(bool manual_smoking)
{
    app_manual_smoking = manual_smoking;
    Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN,
                  manual_smoking ? CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF);
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

    set_manual_label(manual_smoking);
    set_distance_cm(0U);

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

    error = sen66_apply_voc_tuning();
    if (NO_ERROR != error)
    {
        printf("[SEN66] VOC tuning failed status=%d; continuing with sensor defaults\r\n", error);
    }

    error = sen66_start_continuous_measurement();
    if (NO_ERROR != error)
    {
        printf("sen66_start_error:%d\r\n", error);
        vTaskSuspend(NULL);
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
        uint16_t distance_encoded = 0U;
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

        wait_sample_period_and_update_label(&manual_smoking, &sw1_was_pressed,
                                            &sw1_stable_pressed_count, &sample_period_ms);

        error = sen66_read_measured_values_as_integers(&pm1p0, &pm2p5, &pm4p0, &pm10p0,
                                                       &humidity, &temperature, &voc, &nox, &co2);
        if (NO_ERROR != error)
        {
            printf("sen66_read_error:%d\r\n", error);
            continue;
        }

        manual_smoking = app_manual_smoking;
        format_u16_scaled_1(pm1p0, pm1p0_text, sizeof(pm1p0_text));
        format_u16_scaled_1(pm2p5, pm2p5_text, sizeof(pm2p5_text));
        format_u16_scaled_1(pm4p0, pm4p0_text, sizeof(pm4p0_text));
        format_u16_scaled_1(pm10p0, pm10p0_text, sizeof(pm10p0_text));
        format_i16_scaled_2(humidity, 100, humidity_text, sizeof(humidity_text));
        format_i16_scaled_2(temperature, 200, temperature_text, sizeof(temperature_text));
        format_i16_scaled_1(voc, voc_text, sizeof(voc_text));
        format_i16_scaled_1(nox, nox_text, sizeof(nox_text));
        format_unsigned_distance_m(0U, distance_text, sizeof(distance_text));

        snprintf(serial_line, sizeof(serial_line),
                 "PM1.0: %s  PM2.5: %s  PM4.0: %s  PM10.0: %s  Humidity: %s  Temperature: %s  VOC: %s  NOx: %s  CO2: %u  Distance: %s  Manual Label: %s\r\n",
                 pm1p0_text, pm2p5_text, pm4p0_text, pm10p0_text,
                 humidity_text, temperature_text, voc_text, nox_text, co2, distance_text,
                 manual_smoking ? MANUAL_LABEL_SMOKING : MANUAL_LABEL_NO_SMOKING);
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
        put_u16_le(ble_packet, 20U, distance_encoded);
#if defined(DEBUG)
        printf("[BLE] distance encoded=%u unit=mm display_unit=m\r\n",
               (unsigned int)distance_encoded);
#endif
        app_ble_iaq_notify_sensor_packet(ble_packet);
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

    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    if (pdPASS != xTaskCreate(task_sen66_stream, "SEN66", SEN66_TASK_STACK_SIZE,
                              NULL, SEN66_TASK_PRIORITY, NULL))
    {
        handle_app_error();
    }

    vTaskStartScheduler();

    handle_app_error();
}

/* [] END OF FILE */

