#include "app_cm55_events.h"

void app_cm55_send_event(app_event_t event)
{
#if APP_CM55_STARTUP_DIAGNOSTICS
    switch (event)
    {
        case APP_EVENT_CM55_DISPLAY_INIT_ENTERED:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_DISPLAY_INIT_ENTER);
            break;
        case APP_EVENT_CM55_DISPLAY_INIT_DONE:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_DISPLAY_INIT_DONE);
            break;
        case APP_EVENT_CM55_USB_INIT_ENTERED:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_USB_INIT_ENTER);
            break;
        case APP_EVENT_CM55_USB_INIT_DONE:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_USB_INIT_DONE);
            break;
        case APP_EVENT_CM55_CAMERA_INIT_ENTERED:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_CAMERA_INIT_ENTER);
            break;
        case APP_EVENT_CM55_CAMERA_INIT_DONE:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_CAMERA_INIT_DONE);
            break;
        case APP_EVENT_CM55_MODEL_INIT_ENTERED:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_MODEL_INIT_ENTER);
            break;
        case APP_EVENT_CM55_MODEL_INIT_DONE:
            app_cm55_boot_mark(APP_CM55_BOOT_MARK_MODEL_INIT_DONE);
            break;
        default:
            break;
    }
#endif
    (void)app_event_try_send(event);
}

void app_cm55_update_smoke_state(bool smoke_detected)
{
    static bool previous_smoke_detected;

    if (smoke_detected && !previous_smoke_detected)
    {
        app_cm55_send_event(APP_EVENT_SMOKE_DETECTED);
    }

    previous_smoke_detected = smoke_detected;
}

void app_cm55_startup_diagnostic(app_event_t event, app_cm55_boot_mark_t mark)
{
#if APP_CM55_STARTUP_DIAGNOSTICS
    app_cm55_boot_mark(mark);
    app_cm55_send_event(event);
#else
    (void)event;
    (void)mark;
#endif
}
