#ifndef APP_BLE_IAQ_H
#define APP_BLE_IAQ_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#define APP_BLE_IAQ_PACKET_LEN          (23U)
#define APP_BLE_IAQ_MANUAL_LABEL_OFFSET (22U)
#define APP_BLE_IAQ_MANUAL_LABEL_NO_SMOKING (0U)
#define APP_BLE_IAQ_MANUAL_LABEL_SMOKING    (1U)

typedef enum
{
    APP_BLE_IAQ_CMD_START_STREAMING = 0x01,
    APP_BLE_IAQ_CMD_STOP_STREAMING = 0x02,
    APP_BLE_IAQ_CMD_SET_INTERVAL_MS = 0x03,
    APP_BLE_IAQ_CMD_IMMEDIATE_SAMPLE = 0x04,
    APP_BLE_IAQ_CMD_SET_DISTANCE_CM = 0x05,
    APP_BLE_IAQ_CMD_SET_MANUAL_LABEL = 0x06,
} app_ble_iaq_command_id_t;

typedef struct
{
    app_ble_iaq_command_id_t id;
    uint16_t value_u16;
    bool value_bool;
} app_ble_iaq_command_t;

QueueHandle_t app_ble_iaq_get_command_queue(void);
bool app_ble_iaq_receive_command(app_ble_iaq_command_t *command, TickType_t ticks_to_wait);
void app_ble_iaq_init(void);
void app_ble_iaq_set_streaming_enabled(bool enabled);
bool app_ble_iaq_is_streaming_enabled(void);
void app_ble_iaq_notify_sensor_packet(const uint8_t packet[APP_BLE_IAQ_PACKET_LEN]);

#endif
