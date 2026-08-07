/*****************************************************************************
 * File Name        : shared_mem.h
 *
 * Description      : This is the application specification header file
 *
 * Related Document : See README.md
 *
********************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SHARDED_MEM_VALID    (0xA5A5A5A5)
#define APP_EVENT_QUEUE_MAGIC (0x45565155UL)
#define APP_EVENT_QUEUE_DEPTH (16U)
#define APP_CM55_BOOT_STATE_MAGIC (0x4235554DUL)
#define APP_CM55_BOOT_NONE (0x00000000UL)
#define APP_CM55_BOOT_RESET (0x55000001UL)
#define APP_CM55_BOOT_MAIN (0x55000003UL)
#define APP_CM55_BOOT_RTOS (0x55000004UL)
#define APP_CM55_BOOT_APP (0x55000005UL)
#define APP_CM55_BOOT_FAULT (0x55FF0001UL)

#ifndef APP_CM55_STARTUP_DIAGNOSTICS
#define APP_CM55_STARTUP_DIAGNOSTICS (0)
#endif

typedef struct
{
    uint32_t is_valid;
    uint32_t app_boot_address;
} oob_shared_data_t;

typedef enum
{
    APP_EVENT_CAMERA_INITIALIZED = 0,
    APP_EVENT_CAMERA_FAILED,
    APP_EVENT_DISPLAY_INITIALIZED,
    APP_EVENT_DISPLAY_FAILED,
    APP_EVENT_MODEL_INITIALIZED,
    APP_EVENT_MODEL_FAILED,
    APP_EVENT_INFERENCE_FAILED,
    APP_EVENT_SMOKE_DETECTED,
    APP_EVENT_CM55_BOOT_ENTERED,
    APP_EVENT_CM55_MAIN_ENTERED,
    APP_EVENT_CM55_RTOS_STARTED,
    APP_EVENT_CM55_APP_INIT_ENTERED,
    APP_EVENT_CM55_DISPLAY_INIT_ENTERED,
    APP_EVENT_CM55_USB_INIT_ENTERED,
    APP_EVENT_CM55_CAMERA_INIT_ENTERED,
    APP_EVENT_CM55_MODEL_INIT_ENTERED,
    APP_EVENT_CM55_VIDEO_TASK_STARTED
} app_event_t;

typedef struct
{
    uint32_t magic;
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    volatile uint32_t dropped_count;
    volatile uint8_t events[APP_EVENT_QUEUE_DEPTH];
} app_event_queue_t;

typedef struct
{
    uint32_t magic;
    volatile uint32_t state;
    volatile uint32_t fault;
    volatile uint32_t boot_entered;
    volatile uint32_t main_entered;
    volatile uint32_t rtos_started;
    volatile uint32_t app_init_entered;
} app_cm55_boot_state_t;

typedef enum
{
    APP_CM55_BOOT_MARK_BOOT = 0,
    APP_CM55_BOOT_MARK_MAIN,
    APP_CM55_BOOT_MARK_RTOS,
    APP_CM55_BOOT_MARK_APP_INIT
} app_cm55_boot_mark_t;

void shared_mem_write(oob_shared_data_t *cfg);
void shared_mem_read(oob_shared_data_t *cfg);
void app_event_queue_init(void);
bool app_event_try_send(app_event_t event);
bool app_event_try_receive(app_event_t *event);
void app_cm55_boot_state_init(void);
void app_cm55_boot_mark(app_cm55_boot_mark_t mark);
void app_cm55_boot_fault(uint32_t fault);
void app_cm55_boot_state_read(app_cm55_boot_state_t *state);
uintptr_t app_shared_data_address(void);
size_t app_shared_data_size(void);

#endif

/* [] END OF FILE */
