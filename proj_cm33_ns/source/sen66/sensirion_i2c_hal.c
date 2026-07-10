#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"

#include "cybsp.h"
#include "mtb_hal.h"

#define SEN66_I2C_TIMEOUT_MS (1000U)
#define SEN66_I2C_FREQ_HZ    (100000U)

static mtb_hal_i2c_t sen66_i2c_obj;
static cy_stc_scb_i2c_context_t sen66_i2c_context;
static bool sen66_i2c_initialized = false;

int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx)
{
    CY_UNUSED_PARAMETER(bus_idx);
    return NO_ERROR;
}

void sensirion_i2c_hal_init(void)
{
    cy_rslt_t result;
    mtb_hal_i2c_cfg_t i2c_cfg =
    {
        .is_target = MTB_HAL_I2C_MODE_CONTROLLER,
        .address = 0U,
        .frequency_hz = SEN66_I2C_FREQ_HZ,
        .address_mask = MTB_HAL_I2C_DEFAULT_ADDR_MASK,
        .enable_address_callback = false
    };

    if (sen66_i2c_initialized)
    {
        return;
    }

    result = (cy_rslt_t)Cy_SCB_I2C_Init(CYBSP_I2C_CAM_CONTROLLER_HW,
                                        &CYBSP_I2C_CAM_CONTROLLER_config,
                                        &sen66_i2c_context);
    if (CY_RSLT_SUCCESS != result)
    {
        return;
    }

    Cy_SCB_I2C_Enable(CYBSP_I2C_CAM_CONTROLLER_HW);

    result = mtb_hal_i2c_setup(&sen66_i2c_obj,
                               &CYBSP_I2C_CAM_CONTROLLER_hal_config,
                               &sen66_i2c_context,
                               NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        return;
    }

    result = mtb_hal_i2c_configure(&sen66_i2c_obj, &i2c_cfg);
    if (CY_RSLT_SUCCESS == result)
    {
        sen66_i2c_initialized = true;
    }
}

void sensirion_i2c_hal_free(void)
{
    if (sen66_i2c_initialized)
    {
        Cy_SCB_I2C_Disable(CYBSP_I2C_CAM_CONTROLLER_HW, &sen66_i2c_context);
        sen66_i2c_initialized = false;
    }
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count)
{
    cy_rslt_t result;

    if (!sen66_i2c_initialized)
    {
        return NOT_IMPLEMENTED_ERROR;
    }

    result = mtb_hal_i2c_controller_read(&sen66_i2c_obj, address, data, count,
                                         SEN66_I2C_TIMEOUT_MS, true);
    return (CY_RSLT_SUCCESS == result) ? NO_ERROR : (-1);
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count)
{
    cy_rslt_t result;

    if (!sen66_i2c_initialized)
    {
        return NOT_IMPLEMENTED_ERROR;
    }

    result = mtb_hal_i2c_controller_write(&sen66_i2c_obj, address, data, count,
                                          SEN66_I2C_TIMEOUT_MS, true);
    return (CY_RSLT_SUCCESS == result) ? NO_ERROR : (-1);
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds)
{
    uint32_t mseconds = (useconds + 999U) / 1000U;

    if (mseconds == 0U)
    {
        mseconds = 1U;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskDelay(pdMS_TO_TICKS(mseconds));
    }
    else
    {
        mtb_hal_system_delay_ms(mseconds);
    }
}



