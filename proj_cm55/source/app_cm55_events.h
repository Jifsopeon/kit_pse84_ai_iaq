#ifndef APP_CM55_EVENTS_H
#define APP_CM55_EVENTS_H

#include <stdbool.h>
#include "shared_mem.h"

void app_cm55_send_event(app_event_t event);
void app_cm55_update_smoke_state(bool smoke_detected);
void app_cm55_startup_diagnostic(app_event_t event, app_cm55_boot_mark_t mark);

#endif
