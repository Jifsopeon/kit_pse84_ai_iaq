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
#define APP_CM55_BOOT_RESET_HANDLER (0x55000001UL)
#define APP_CM55_BOOT_SYSTEM_INIT_ENTER (0x55000002UL)
#define APP_CM55_BOOT_SYSTEM_INIT_DONE (0x55000003UL)
#define APP_CM55_BOOT_CRT_INIT_ENTER (0x55000004UL)
#define APP_CM55_BOOT_MAIN_ENTER (0x55000005UL)
#define APP_CM55_BOOT_PLATFORM_INIT_ENTER (0x55000006UL)
#define APP_CM55_BOOT_PLATFORM_INIT_DONE (0x55000007UL)
#define APP_CM55_BOOT_PSRAM_INIT_ENTER (0x55000008UL)
#define APP_CM55_BOOT_PSRAM_INIT_DONE (0x55000009UL)
#define APP_CM55_BOOT_RTOS_CREATE (0x5500000AUL)
#define APP_CM55_BOOT_SCHEDULER_STARTED (0x5500000BUL)
#define APP_CM55_BOOT_APP_INIT_ENTER (0x5500000CUL)
#define APP_CM55_BOOT_DISPLAY_INIT_ENTER (0x5500000DUL)
#define APP_CM55_BOOT_USB_INIT_ENTER (0x5500000EUL)
#define APP_CM55_BOOT_MODEL_INIT_ENTER (0x5500000FUL)
#define APP_CM55_BOOT_DISPLAY_INIT_DONE (0x55000010UL)
#define APP_CM55_BOOT_USB_INIT_DONE (0x55000011UL)
#define APP_CM55_BOOT_MODEL_INIT_DONE (0x55000012UL)
#define APP_CM55_BOOT_CAMERA_INIT_ENTER (0x55000013UL)
#define APP_CM55_BOOT_CAMERA_INIT_DONE (0x55000014UL)
#define APP_CM55_BOOT_FAULT (0x55FF0000UL)

#define APP_CM55_FAULT_NONE (0x00000000UL)
#define APP_CM55_FAULT_HARDFAULT (0x55FF0001UL)
#define APP_CM55_FAULT_MEMMANAGE (0x55FF0002UL)
#define APP_CM55_FAULT_BUSFAULT (0x55FF0003UL)
#define APP_CM55_FAULT_USAGEFAULT (0x55FF0004UL)
#define APP_CM55_FAULT_DEBUGMON (0x55FF0005UL)

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
    APP_EVENT_CM55_DISPLAY_INIT_DONE,
    APP_EVENT_CM55_USB_INIT_ENTERED,
    APP_EVENT_CM55_USB_INIT_DONE,
    APP_EVENT_CM55_CAMERA_INIT_ENTERED,
    APP_EVENT_CM55_CAMERA_INIT_DONE,
    APP_EVENT_CM55_MODEL_INIT_ENTERED,
    APP_EVENT_CM55_MODEL_INIT_DONE,
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
    volatile uint32_t last_state;
    volatile uint32_t fault;
    volatile uint32_t exc_return;
    volatile uint32_t msp;
    volatile uint32_t psp;
    volatile uint32_t control;
    volatile uint32_t primask;
    volatile uint32_t basepri;
    volatile uint32_t faultmask;
    volatile uint32_t msplim;
    volatile uint32_t psplim;
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;
    volatile uint32_t pc;
    volatile uint32_t xpsr;
    volatile uint32_t cfsr;
    volatile uint32_t hfsr;
    volatile uint32_t dfsr;
    volatile uint32_t afsr;
    volatile uint32_t mmfar;
    volatile uint32_t bfar;
    volatile uint32_t shcsr;
    volatile uint32_t icsr;
    volatile uint32_t vtor;
    volatile uint32_t boot_entered;
    volatile uint32_t main_entered;
    volatile uint32_t rtos_started;
    volatile uint32_t app_init_entered;
} app_cm55_boot_state_t;

typedef enum
{
    APP_CM55_BOOT_MARK_RESET_HANDLER = 0,
    APP_CM55_BOOT_MARK_SYSTEM_INIT_ENTER,
    APP_CM55_BOOT_MARK_SYSTEM_INIT_DONE,
    APP_CM55_BOOT_MARK_CRT_INIT_ENTER,
    APP_CM55_BOOT_MARK_MAIN_ENTER,
    APP_CM55_BOOT_MARK_PLATFORM_INIT_ENTER,
    APP_CM55_BOOT_MARK_PLATFORM_INIT_DONE,
    APP_CM55_BOOT_MARK_PSRAM_INIT_ENTER,
    APP_CM55_BOOT_MARK_PSRAM_INIT_DONE,
    APP_CM55_BOOT_MARK_RTOS_CREATE,
    APP_CM55_BOOT_MARK_SCHEDULER_STARTED,
    APP_CM55_BOOT_MARK_APP_INIT_ENTER,
    APP_CM55_BOOT_MARK_DISPLAY_INIT_ENTER,
    APP_CM55_BOOT_MARK_USB_INIT_ENTER,
    APP_CM55_BOOT_MARK_MODEL_INIT_ENTER,
    APP_CM55_BOOT_MARK_DISPLAY_INIT_DONE,
    APP_CM55_BOOT_MARK_USB_INIT_DONE,
    APP_CM55_BOOT_MARK_MODEL_INIT_DONE,
    APP_CM55_BOOT_MARK_CAMERA_INIT_ENTER,
    APP_CM55_BOOT_MARK_CAMERA_INIT_DONE
} app_cm55_boot_mark_t;

typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} app_cm55_exception_frame_t;

void shared_mem_write(oob_shared_data_t *cfg);
void shared_mem_read(oob_shared_data_t *cfg);
void app_event_queue_init(void);
bool app_event_try_send(app_event_t event);
bool app_event_try_receive(app_event_t *event);
void app_cm55_boot_state_init(void);
void app_cm55_boot_mark(app_cm55_boot_mark_t mark);
void app_cm55_boot_fault(uint32_t fault);
void app_cm55_fault_capture(uint32_t fault, uint32_t exc_return, const app_cm55_exception_frame_t *frame);
void app_cm55_boot_state_read(app_cm55_boot_state_t *state);
uintptr_t app_shared_data_address(void);
size_t app_shared_data_size(void);
uintptr_t app_shared_data_end_address(void);

#endif

/* [] END OF FILE */
