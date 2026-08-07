/*****************************************************************************
 * File Name        : shared_mem.c
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

#include "shared_mem.h"
#include "cybsp.h"
#include "cy_utils.h"
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SHARED_MEM_CACHE_LINE_SIZE (32UL)

typedef struct
{
    oob_shared_data_t oob;
    app_cm55_boot_state_t boot_state;
    app_event_queue_t events;
} app_shared_data_t;

#define SRAM1_SHARED_DATA_ADDR     (CYMEM_CM33_0_m55_allocatable_shared_START + CYMEM_CM33_0_m55_allocatable_shared_SIZE - (sizeof(app_shared_data_t)))

static app_shared_data_t *app_shared_data(void)
{
    return (app_shared_data_t *)SRAM1_SHARED_DATA_ADDR;
}

uintptr_t app_shared_data_address(void)
{
    return (uintptr_t)app_shared_data();
}

size_t app_shared_data_size(void)
{
    return sizeof(app_shared_data_t);
}

static void shared_cache_clean_invalidate(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start = ((uintptr_t)app_shared_data()) & ~(uintptr_t)(SHARED_MEM_CACHE_LINE_SIZE - 1UL);
    uintptr_t end = ((uintptr_t)app_shared_data() + sizeof(app_shared_data_t) + SHARED_MEM_CACHE_LINE_SIZE - 1UL) &
                    ~(uintptr_t)(SHARED_MEM_CACHE_LINE_SIZE - 1UL);
    SCB_CleanInvalidateDCache_by_Addr((void *)start, (int32_t)(end - start));
#endif
    __DMB();
}

static void shared_cache_clean(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start = ((uintptr_t)app_shared_data()) & ~(uintptr_t)(SHARED_MEM_CACHE_LINE_SIZE - 1UL);
    uintptr_t end = ((uintptr_t)app_shared_data() + sizeof(app_shared_data_t) + SHARED_MEM_CACHE_LINE_SIZE - 1UL) &
                    ~(uintptr_t)(SHARED_MEM_CACHE_LINE_SIZE - 1UL);
    SCB_CleanDCache_by_Addr((void *)start, (int32_t)(end - start));
#endif
    __DMB();
}


void shared_mem_write(oob_shared_data_t *cfg)
{
    char *src = (char *)cfg;
    char *dst = (char *)&app_shared_data()->oob;

    memcpy(dst, src, sizeof(oob_shared_data_t));
    shared_cache_clean();
}

void shared_mem_read(oob_shared_data_t *cfg)
{
    shared_cache_clean_invalidate();
    char *src = (char *)&app_shared_data()->oob;
    char *dst = (char *)cfg;

    memcpy(dst, src, sizeof(oob_shared_data_t));
}

void app_event_queue_init(void)
{
    app_event_queue_t *queue = &app_shared_data()->events;

    memset(queue, 0, sizeof(*queue));
    queue->magic = APP_EVENT_QUEUE_MAGIC;
    shared_cache_clean();
}

void app_cm55_boot_state_init(void)
{
    app_cm55_boot_state_t *state = &app_shared_data()->boot_state;

    memset(state, 0, sizeof(*state));
    state->magic = APP_CM55_BOOT_STATE_MAGIC;
    shared_cache_clean();
}

void app_cm55_boot_mark(app_cm55_boot_mark_t mark)
{
    app_cm55_boot_state_t *state = &app_shared_data()->boot_state;

    shared_cache_clean_invalidate();
    if (state->magic != APP_CM55_BOOT_STATE_MAGIC)
    {
        state->magic = APP_CM55_BOOT_STATE_MAGIC;
    }

    switch (mark)
    {
        case APP_CM55_BOOT_MARK_BOOT:
            state->state = APP_CM55_BOOT_RESET;
            state->boot_entered++;
            break;
        case APP_CM55_BOOT_MARK_MAIN:
            state->state = APP_CM55_BOOT_MAIN;
            state->main_entered++;
            break;
        case APP_CM55_BOOT_MARK_RTOS:
            state->state = APP_CM55_BOOT_RTOS;
            state->rtos_started++;
            break;
        case APP_CM55_BOOT_MARK_APP_INIT:
            state->state = APP_CM55_BOOT_APP;
            state->app_init_entered++;
            break;
        default:
            break;
    }

    shared_cache_clean();
}

void app_cm55_boot_fault(uint32_t fault)
{
    app_cm55_boot_state_t *state = &app_shared_data()->boot_state;

    shared_cache_clean_invalidate();
    if (state->magic != APP_CM55_BOOT_STATE_MAGIC)
    {
        state->magic = APP_CM55_BOOT_STATE_MAGIC;
    }
    state->state = APP_CM55_BOOT_FAULT;
    state->fault = fault;
    shared_cache_clean();
}

void app_cm55_boot_state_read(app_cm55_boot_state_t *state)
{
    if (NULL == state)
    {
        return;
    }

    shared_cache_clean_invalidate();
    memcpy(state, &app_shared_data()->boot_state, sizeof(*state));
}

bool app_event_try_send(app_event_t event)
{
    app_event_queue_t *queue = &app_shared_data()->events;

    shared_cache_clean_invalidate();
    if (queue->magic != APP_EVENT_QUEUE_MAGIC)
    {
        app_event_queue_init();
    }

    uint32_t write_index = queue->write_index;
    uint32_t read_index = queue->read_index;
    uint32_t next_write = (write_index + 1UL) % APP_EVENT_QUEUE_DEPTH;

    if (next_write == read_index)
    {
        queue->dropped_count++;
        shared_cache_clean();
        return false;
    }

    queue->events[write_index] = (uint8_t)event;
    queue->write_index = next_write;
    shared_cache_clean();
    return true;
}

bool app_event_try_receive(app_event_t *event)
{
    app_event_queue_t *queue = &app_shared_data()->events;

    if (NULL == event)
    {
        return false;
    }

    shared_cache_clean_invalidate();
    if (queue->magic != APP_EVENT_QUEUE_MAGIC)
    {
        return false;
    }

    uint32_t read_index = queue->read_index;
    if (read_index == queue->write_index)
    {
        return false;
    }

    *event = (app_event_t)queue->events[read_index];
    queue->read_index = (read_index + 1UL) % APP_EVENT_QUEUE_DEPTH;
    shared_cache_clean();
    return true;
}
