#include "app_cm55_events.h"

void app_cm55_send_event(app_event_t event)
{
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
