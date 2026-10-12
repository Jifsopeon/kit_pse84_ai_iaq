/*****************************************************************************
 * File Name: usbh_log_stubs.c
 *
 * Description: Non-UART USB host log/panic hooks for the CM55 project.
 *****************************************************************************/

#include "app_cm55_events.h"
#include "cy_syslib.h"

void USBH_Panic(const char *s)
{
    (void)s;
    app_cm55_send_event(APP_EVENT_CAMERA_FAILED);
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}

void USBH_Log(const char *s)
{
    (void)s;
}

void USBH_Warn(const char *s)
{
    (void)s;
}
