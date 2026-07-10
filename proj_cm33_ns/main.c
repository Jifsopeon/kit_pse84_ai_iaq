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
#define SEN66_SAMPLE_PERIOD_MS              (1000U)
#define SW1_POLL_PERIOD_MS                  (20U)
#define SW1_DEBOUNCE_POLLS                  (3U)
#define MANUAL_LABEL_NO_SMOKING             "no smoking"
#define MANUAL_LABEL_SMOKING                "smoking"

#if defined(CYBSP_CM33_LPTIMER_0)
static mtb_hal_lptimer_t lptimer_obj;
#endif

#if defined(CYBSP_RTC)
static mtb_hal_rtc_t rtc_obj;
#endif

static cy_stc_scb_uart_context_t debug_uart_context;
static mtb_hal_uart_t debug_uart_obj;

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

static void print_signed_fixed_2(int16_t value)
{
    int16_t whole = value / 100;
    int16_t fraction = value % 100;

    if (fraction < 0)
    {
        fraction = (int16_t)-fraction;
    }

    printf("%d.%02d", whole, fraction);
}

static bool sw1_is_pressed(void)
{
    return (0U == Cy_GPIO_Read(CYBSP_SW1_PORT, CYBSP_SW1_PIN));
}

static void set_manual_label_led(bool manual_smoking)
{
    Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN,
                  manual_smoking ? CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF);
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
            set_manual_label_led(*manual_smoking);
            *sw1_was_pressed = true;
        }
    }
    else
    {
        *sw1_stable_pressed_count = 0U;
        *sw1_was_pressed = false;
    }
}

static void wait_sample_period_and_update_label(bool* manual_smoking, bool* sw1_was_pressed,
                                                uint8_t* sw1_stable_pressed_count)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < SEN66_SAMPLE_PERIOD_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(SW1_POLL_PERIOD_MS));
        update_manual_label(manual_smoking, sw1_was_pressed, sw1_stable_pressed_count);
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

    set_manual_label_led(manual_smoking);

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

    error = sen66_start_continuous_measurement();
    if (NO_ERROR != error)
    {
        printf("sen66_start_error:%d\r\n", error);
        vTaskSuspend(NULL);
    }

    wait_sample_period_and_update_label(&manual_smoking, &sw1_was_pressed, &sw1_stable_pressed_count);

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

        wait_sample_period_and_update_label(&manual_smoking, &sw1_was_pressed, &sw1_stable_pressed_count);

        error = sen66_read_measured_values_as_integers(&pm1p0, &pm2p5, &pm4p0, &pm10p0,
                                                       &humidity, &temperature, &voc, &nox, &co2);
        if (NO_ERROR != error)
        {
            printf("sen66_read_error:%d\r\n", error);
            continue;
        }

        printf("PM1.0: %u  PM2.5: %u  PM4.0: %u  PM10.0: %u  Humidity: ",
               pm1p0, pm2p5, pm4p0, pm10p0);
        print_signed_fixed_2(humidity);
        printf("  Temperature: ");
        print_signed_fixed_2(temperature);
        printf("  VOC: %d  NOx: %d  CO2: %u  Manual Label: %s\r\n",
               voc, nox, co2,
               manual_smoking ? MANUAL_LABEL_SMOKING : MANUAL_LABEL_NO_SMOKING);
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

    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    __enable_irq();

    if (pdPASS != xTaskCreate(task_sen66_stream, "SEN66", SEN66_TASK_STACK_SIZE,
                              NULL, SEN66_TASK_PRIORITY, NULL))
    {
        handle_app_error();
    }

    vTaskStartScheduler();

    handle_app_error();
}

/* [] END OF FILE */

