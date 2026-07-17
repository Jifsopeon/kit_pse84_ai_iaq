#include "app_ble_iaq.h"

#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "cybsp.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_memory.h"

#include "FreeRTOS.h"
#include "queue.h"

#include <stdio.h>
#include <string.h>

#define APP_BLE_DEBUG_ENABLED              (1U)
#define APP_BLE_DEBUG(fmt, ...)            do { if (APP_BLE_DEBUG_ENABLED) { printf("[BLE] " fmt "\r\n", ##__VA_ARGS__); } } while (0)
#define APP_BLE_COMMAND_QUEUE_DEPTH        (8U)
#define APP_BLE_IAQ_LOCAL_ATT_MTU          (26U)
#define APP_BLE_ATT_NOTIFICATION_OVERHEAD  (3U)
#define APP_BLE_IAQ_NOTIFY_VALUE_MAX       (APP_BLE_IAQ_LOCAL_ATT_MTU - APP_BLE_ATT_NOTIFICATION_OVERHEAD)
#define APP_BLE_GATT_RSP_POOL_BUFFER_SIZE  (32U)
#define APP_BLE_GATT_RSP_POOL_BUFFER_COUNT (4U)
#define APP_BLE_GATT_RSP_POOL_OVERHEAD     (0U)

static uint16_t conn_id;
static QueueHandle_t command_queue;
static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[APP_BLE_COMMAND_QUEUE_DEPTH * sizeof(app_ble_iaq_command_t)];
static volatile bool streaming_enabled = true;
static uint8_t gatt_response_pool[APP_BLE_GATT_RSP_POOL_BUFFER_COUNT][APP_BLE_GATT_RSP_POOL_BUFFER_SIZE];
static bool gatt_response_pool_in_use[APP_BLE_GATT_RSP_POOL_BUFFER_COUNT];
static uint32_t gatt_response_pool_outstanding;
static uint32_t gatt_response_pool_high_watermark;
typedef void (*pfn_free_buffer_t)(uint8_t *);

static const char *app_gatt_status_name(wiced_bt_gatt_status_t status)
{
    switch (status)
    {
        case WICED_BT_GATT_SUCCESS: return "WICED_BT_GATT_SUCCESS";
        case WICED_BT_GATT_INVALID_HANDLE: return "WICED_BT_GATT_INVALID_HANDLE";
        case WICED_BT_GATT_READ_NOT_PERMIT: return "WICED_BT_GATT_READ_NOT_PERMIT";
        case WICED_BT_GATT_WRITE_NOT_PERMIT: return "WICED_BT_GATT_WRITE_NOT_PERMIT";
        case WICED_BT_GATT_INVALID_PDU: return "WICED_BT_GATT_INVALID_PDU";
        case WICED_BT_GATT_INSUF_AUTHENTICATION: return "WICED_BT_GATT_INSUF_AUTHENTICATION";
        case WICED_BT_GATT_REQ_NOT_SUPPORTED: return "WICED_BT_GATT_REQ_NOT_SUPPORTED";
        case WICED_BT_GATT_INVALID_OFFSET: return "WICED_BT_GATT_INVALID_OFFSET";
        case WICED_BT_GATT_INSUF_AUTHORIZATION: return "WICED_BT_GATT_INSUF_AUTHORIZATION";
        case WICED_BT_GATT_PREPARE_Q_FULL: return "WICED_BT_GATT_PREPARE_Q_FULL";
        case WICED_BT_GATT_ATTRIBUTE_NOT_FOUND: return "WICED_BT_GATT_ATTRIBUTE_NOT_FOUND";
        case WICED_BT_GATT_NOT_LONG: return "WICED_BT_GATT_NOT_LONG";
        case WICED_BT_GATT_INSUF_KEY_SIZE: return "WICED_BT_GATT_INSUF_KEY_SIZE";
        case WICED_BT_GATT_INVALID_ATTR_LEN: return "WICED_BT_GATT_INVALID_ATTR_LEN";
        case WICED_BT_GATT_ERR_UNLIKELY: return "WICED_BT_GATT_ERR_UNLIKELY";
        case WICED_BT_GATT_INSUF_ENCRYPTION: return "WICED_BT_GATT_INSUF_ENCRYPTION";
        case WICED_BT_GATT_UNSUPPORT_GRP_TYPE: return "WICED_BT_GATT_UNSUPPORT_GRP_TYPE";
        case WICED_BT_GATT_INSUF_RESOURCE: return "WICED_BT_GATT_INSUF_RESOURCE";
        case WICED_BT_GATT_DATABASE_OUT_OF_SYNC: return "WICED_BT_GATT_DATABASE_OUT_OF_SYNC";
        case WICED_BT_GATT_VALUE_NOT_ALLOWED: return "WICED_BT_GATT_VALUE_NOT_ALLOWED";
        case WICED_BT_GATT_BUSY: return "WICED_BT_GATT_BUSY";
        case WICED_BT_GATT_CONGESTED: return "WICED_BT_GATT_CONGESTED";
        case WICED_BT_GATT_INVALID_CONNECTION_ID: return "WICED_BT_GATT_INVALID_CONNECTION_ID";
        default: return "WICED_BT_GATT_STATUS_UNKNOWN";
    }
}

static void app_hex_encode(const uint8_t *data, uint16_t length, char *output, uint16_t output_length)
{
    static const char hex[] = "0123456789ABCDEF";
    uint16_t out = 0U;

    for (uint16_t i = 0U; (i < length) && ((out + 3U) < output_length); i++)
    {
        output[out++] = hex[(data[i] >> 4) & 0x0F];
        output[out++] = hex[data[i] & 0x0F];
        if ((i + 1U) < length)
        {
            output[out++] = ' ';
        }
    }

    if (out < output_length)
    {
        output[out] = '\0';
    }
}

static uint32_t app_gatt_response_pool_free_count(void)
{
    uint32_t free_count = 0U;

    taskENTER_CRITICAL();
    for (uint32_t i = 0U; i < APP_BLE_GATT_RSP_POOL_BUFFER_COUNT; i++)
    {
        if (!gatt_response_pool_in_use[i])
        {
            free_count++;
        }
    }
    taskEXIT_CRITICAL();

    return free_count;
}

static void *app_gatt_response_buffer_alloc(uint16_t len_requested)
{
    void *buffer = NULL;
    uint32_t selected_index = APP_BLE_GATT_RSP_POOL_BUFFER_COUNT;
    uint32_t free_count = 0U;
    uint32_t outstanding = 0U;
    uint32_t high_watermark = 0U;

    taskENTER_CRITICAL();
    if (len_requested <= APP_BLE_GATT_RSP_POOL_BUFFER_SIZE)
    {
        for (uint32_t i = 0U; i < APP_BLE_GATT_RSP_POOL_BUFFER_COUNT; i++)
        {
            if (!gatt_response_pool_in_use[i])
            {
                gatt_response_pool_in_use[i] = true;
                buffer = gatt_response_pool[i];
                selected_index = i;
                gatt_response_pool_outstanding++;
                if (gatt_response_pool_outstanding > gatt_response_pool_high_watermark)
                {
                    gatt_response_pool_high_watermark = gatt_response_pool_outstanding;
                }
                break;
            }
        }
    }

    for (uint32_t i = 0U; i < APP_BLE_GATT_RSP_POOL_BUFFER_COUNT; i++)
    {
        if (!gatt_response_pool_in_use[i])
        {
            free_count++;
        }
    }
    outstanding = gatt_response_pool_outstanding;
    high_watermark = gatt_response_pool_high_watermark;
    taskEXIT_CRITICAL();

    APP_BLE_DEBUG("response buffer request len=%u %s pool=app_gatt_rsp capacity=%u overhead=%u index=%ld outstanding=%lu high=%lu free=%lu",
                  len_requested,
                  (NULL == buffer) ? "failed" : "ok",
                  APP_BLE_GATT_RSP_POOL_BUFFER_SIZE,
                  APP_BLE_GATT_RSP_POOL_OVERHEAD,
                  (NULL == buffer) ? -1L : (long)selected_index,
                  (unsigned long)outstanding,
                  (unsigned long)high_watermark,
                  (unsigned long)free_count);

    return buffer;
}

static void app_gatt_response_buffer_free(uint8_t *buffer)
{
    bool released = false;
    uint32_t released_index = APP_BLE_GATT_RSP_POOL_BUFFER_COUNT;
    uint32_t free_count = 0U;
    uint32_t outstanding = 0U;

    taskENTER_CRITICAL();
    for (uint32_t i = 0U; i < APP_BLE_GATT_RSP_POOL_BUFFER_COUNT; i++)
    {
        if (buffer == gatt_response_pool[i])
        {
            if (gatt_response_pool_in_use[i])
            {
                gatt_response_pool_in_use[i] = false;
                if (gatt_response_pool_outstanding > 0U)
                {
                    gatt_response_pool_outstanding--;
                }
                released = true;
            }
            released_index = i;
            break;
        }
    }

    for (uint32_t i = 0U; i < APP_BLE_GATT_RSP_POOL_BUFFER_COUNT; i++)
    {
        if (!gatt_response_pool_in_use[i])
        {
            free_count++;
        }
    }
    outstanding = gatt_response_pool_outstanding;
    taskEXIT_CRITICAL();

    APP_BLE_DEBUG("response buffer release pool=app_gatt_rsp index=%ld result=%s outstanding=%lu free=%lu",
                  (APP_BLE_GATT_RSP_POOL_BUFFER_COUNT == released_index) ? -1L : (long)released_index,
                  released ? "ok" : "ignored",
                  (unsigned long)outstanding,
                  (unsigned long)free_count);
}

static gatt_db_lookup_table_t *app_get_attribute(uint16_t handle)
{
    for (uint16_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return &app_gatt_db_ext_attr_tbl[i];
        }
    }

    return NULL;
}

static wiced_bt_gatt_status_t app_read_handler(uint16_t conn_id_value,
                                               wiced_bt_gatt_opcode_t opcode,
                                               wiced_bt_gatt_read_t *read_data,
                                               uint16_t len_requested)
{
    gatt_db_lookup_table_t *attribute = app_get_attribute(read_data->handle);
    if (NULL == attribute)
    {
        APP_BLE_DEBUG("read invalid handle=0x%04X", read_data->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id_value, opcode, read_data->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    if (read_data->offset >= attribute->cur_len)
    {
        APP_BLE_DEBUG("read invalid offset handle=0x%04X offset=%lu len=%u",
                      read_data->handle, (unsigned long)read_data->offset, attribute->cur_len);
        wiced_bt_gatt_server_send_error_rsp(conn_id_value, opcode, read_data->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    uint16_t length_to_send = (uint16_t)(attribute->cur_len - read_data->offset);
    if (length_to_send > len_requested)
    {
        length_to_send = len_requested;
    }

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id_value, opcode, length_to_send,
                                              attribute->p_data + read_data->offset, NULL);
    APP_BLE_DEBUG("read handle=0x%04X len=%u", read_data->handle, length_to_send);
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t app_write_handler(uint16_t conn_id_value,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *write_data)
{
    CY_UNUSED_PARAMETER(conn_id_value);
    CY_UNUSED_PARAMETER(opcode);

    gatt_db_lookup_table_t *attribute = app_get_attribute(write_data->handle);
    if (NULL == attribute || write_data->val_len > attribute->max_len)
    {
        APP_BLE_DEBUG("write invalid handle=0x%04X len=%u", write_data->handle, write_data->val_len);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    APP_BLE_DEBUG("write handle=0x%04X len=%u", write_data->handle, write_data->val_len);
    memcpy(attribute->p_data, write_data->p_val, write_data->val_len);
    attribute->cur_len = write_data->val_len;

    switch (write_data->handle)
    {
        case HDLD_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG:
            app_iaq_sensor_data_client_char_config[0] = write_data->p_val[0];
            app_iaq_sensor_data_client_char_config[1] = write_data->p_val[1];
            APP_BLE_DEBUG("sensor notifications %s",
                          (app_iaq_sensor_data_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION) ?
                          "enabled" : "disabled");
            break;

        case HDLC_IAQ_COMMAND_VALUE:
            if (write_data->val_len >= 1U)
            {
                app_ble_iaq_command_t command =
                {
                    .id = (app_ble_iaq_command_id_t)write_data->p_val[0],
                    .value_u16 = 0U,
                    .value_bool = false,
                };

                if (write_data->val_len >= 3U)
                {
                    command.value_u16 = (uint16_t)write_data->p_val[1] |
                                        ((uint16_t)write_data->p_val[2] << 8U);
                }
                if (write_data->val_len >= 2U)
                {
                    command.value_bool = (0U != write_data->p_val[1]);
                }

                APP_BLE_DEBUG("command write id=0x%02X value=%u bool=%u",
                              (unsigned int)command.id,
                              command.value_u16,
                              command.value_bool ? 1U : 0U);
                if ((NULL == command_queue) ||
                    (pdPASS != xQueueSend(command_queue, &command, 0U)))
                {
                    APP_BLE_DEBUG("command queue full/unavailable");
                    return WICED_BT_GATT_INSUF_RESOURCE;
                }
            }
            break;

        default:
            break;
    }

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t app_attribute_request_handler(wiced_bt_gatt_attribute_request_t *request)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (request->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            APP_BLE_DEBUG("attribute read opcode=0x%02X", request->opcode);
            result = app_read_handler(request->conn_id, request->opcode,
                                      &request->data.read_req, request->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            APP_BLE_DEBUG("attribute write opcode=0x%02X", request->opcode);
            result = app_write_handler(request->conn_id, request->opcode,
                                       &request->data.write_req);
            if (GATT_REQ_WRITE == request->opcode)
            {
                if (WICED_BT_GATT_SUCCESS == result)
                {
                    wiced_bt_gatt_server_send_write_rsp(request->conn_id, request->opcode,
                                                        request->data.write_req.handle);
                }
                else
                {
                    wiced_bt_gatt_server_send_error_rsp(request->conn_id, request->opcode,
                                                        request->data.write_req.handle, result);
                }
            }
            break;

        case GATT_REQ_MTU:
        {
            uint16_t local_mtu = APP_BLE_IAQ_LOCAL_ATT_MTU;
            uint16_t negotiated_mtu = (request->data.remote_mtu < local_mtu) ?
                                      request->data.remote_mtu : local_mtu;
            APP_BLE_DEBUG("MTU request: remote=%u local=%u negotiated=%u",
                          request->data.remote_mtu,
                          local_mtu,
                          negotiated_mtu);
            result = wiced_bt_gatt_server_send_mtu_rsp(request->conn_id, request->data.remote_mtu,
                                                       local_mtu);
            APP_BLE_DEBUG("MTU response result=%d remote=%u local=%u negotiated=%u",
                          result,
                          request->data.remote_mtu,
                          local_mtu,
                          negotiated_mtu);
            break;
        }

        default:
            break;
    }

    return result;
}

static wiced_bt_gatt_status_t app_gatt_callback(wiced_bt_gatt_evt_t event,
                                                wiced_bt_gatt_event_data_t *data)
{
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            conn_id = data->connection_status.connected ? data->connection_status.conn_id : 0U;
            APP_BLE_DEBUG("%s conn_id=%u reason=%u",
                          data->connection_status.connected ? "connected" : "disconnected",
                          data->connection_status.conn_id,
                          data->connection_status.reason);
            if (!data->connection_status.connected)
            {
                (void)wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                               cy_bt_adv_packet_data);
                wiced_result_t adv_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                                                          BLE_ADDR_PUBLIC, NULL);
                APP_BLE_DEBUG("restart advertising result=%d", adv_result);
            }
            return WICED_BT_GATT_SUCCESS;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            APP_BLE_DEBUG("gatt attribute request opcode=0x%02X", data->attribute_request.opcode);
            return app_attribute_request_handler(&data->attribute_request);

        case GATT_GET_RESPONSE_BUFFER_EVT:
            data->buffer_request.buffer.p_app_rsp_buffer =
                app_gatt_response_buffer_alloc(data->buffer_request.len_requested);
            data->buffer_request.buffer.p_app_ctxt = (void *)app_gatt_response_buffer_free;
            return (NULL == data->buffer_request.buffer.p_app_rsp_buffer) ?
                   WICED_BT_GATT_INSUF_RESOURCE : WICED_BT_GATT_SUCCESS;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)data->buffer_xmitted.p_app_ctxt;
            if (pfn_free)
            {
                pfn_free(data->buffer_xmitted.p_app_data);
            }
            return WICED_BT_GATT_SUCCESS;
        }

        default:
            return WICED_BT_GATT_SUCCESS;
    }
}

static void app_ble_start_advertising(void)
{
    wiced_result_t adv_data_result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                                             cy_bt_adv_packet_data);
    wiced_result_t scan_rsp_result = wiced_bt_ble_set_raw_scan_response_data(CY_BT_SCAN_RESP_PACKET_DATA_SIZE,
                                                                             cy_bt_scan_resp_packet_data);
    wiced_bt_gatt_status_t gatt_register_result = wiced_bt_gatt_register(app_gatt_callback);
    wiced_bt_gatt_status_t gatt_db_result = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    wiced_result_t adv_start_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                                    BLE_ADDR_PUBLIC, NULL);

    APP_BLE_DEBUG("set adv data result=%d", adv_data_result);
    APP_BLE_DEBUG("set scan response result=%d", scan_rsp_result);
    APP_BLE_DEBUG("gatt register result=%d", gatt_register_result);
    APP_BLE_DEBUG("gatt db init result=%d len=%u", gatt_db_result, gatt_database_len);
    APP_BLE_DEBUG("start advertising result=%d name=PSE84-IAQ", adv_start_result);
}

static wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                                 wiced_bt_management_evt_data_t *event_data)
{
    APP_BLE_DEBUG("management event=0x%02X", event);
    if ((BTM_ENABLED_EVT == event) && (WICED_BT_SUCCESS == event_data->enabled.status))
    {
        APP_BLE_DEBUG("stack enabled");
        wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
        app_ble_start_advertising();
    }
    else if (BTM_ENABLED_EVT == event)
    {
        APP_BLE_DEBUG("stack enable failed status=%d", event_data->enabled.status);
    }

    return WICED_BT_SUCCESS;
}

void app_ble_iaq_init(void)
{
    if (NULL == command_queue)
    {
        command_queue = xQueueCreateStatic(APP_BLE_COMMAND_QUEUE_DEPTH,
                                           sizeof(app_ble_iaq_command_t),
                                           command_queue_storage,
                                           &command_queue_control);
    }

    APP_BLE_DEBUG("GATT response pool configured pool=app_gatt_rsp capacity=%u count=%u overhead=%u free=%lu",
                  APP_BLE_GATT_RSP_POOL_BUFFER_SIZE,
                  APP_BLE_GATT_RSP_POOL_BUFFER_COUNT,
                  APP_BLE_GATT_RSP_POOL_OVERHEAD,
                  (unsigned long)app_gatt_response_pool_free_count());
    APP_BLE_DEBUG("BLE config CY_BT_RX_PDU_SIZE=%u local_att_mtu=%u max_notification_value_len=%u sensor_packet_len=%u",
                  CY_BT_RX_PDU_SIZE,
                  APP_BLE_IAQ_LOCAL_ATT_MTU,
                  APP_BLE_IAQ_NOTIFY_VALUE_MAX,
                  APP_BLE_IAQ_PACKET_LEN);
    APP_BLE_DEBUG("stack init start");
    wiced_result_t result = wiced_bt_stack_init(app_bt_management_callback,
                                               &cy_bt_cfg_settings);
    if (WICED_BT_SUCCESS != result)
    {
        APP_BLE_DEBUG("stack init failed result=%d", result);
    }
    else
    {
        APP_BLE_DEBUG("stack init pending enable event");
    }
}

QueueHandle_t app_ble_iaq_get_command_queue(void)
{
    return command_queue;
}

bool app_ble_iaq_receive_command(app_ble_iaq_command_t *command, TickType_t ticks_to_wait)
{
    return (NULL != command_queue) &&
           (pdPASS == xQueueReceive(command_queue, command, ticks_to_wait));
}

void app_ble_iaq_set_streaming_enabled(bool enabled)
{
    streaming_enabled = enabled;
    APP_BLE_DEBUG("streaming %s", enabled ? "enabled" : "disabled");
}

bool app_ble_iaq_is_streaming_enabled(void)
{
    return streaming_enabled;
}

void app_ble_iaq_notify_sensor_packet(const uint8_t packet[APP_BLE_IAQ_PACKET_LEN])
{
    char packet_hex[(APP_BLE_IAQ_PACKET_LEN * 3U) + 1U] = {0};

    memcpy(app_iaq_sensor_data, packet, APP_BLE_IAQ_PACKET_LEN);
    app_get_attribute(HDLC_IAQ_SENSOR_DATA_VALUE)->cur_len = APP_BLE_IAQ_PACKET_LEN;
    app_hex_encode(packet, APP_BLE_IAQ_PACKET_LEN, packet_hex, sizeof(packet_hex));

    if ((0U != conn_id) &&
        (app_iaq_sensor_data_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION) &&
        streaming_enabled)
    {
        APP_BLE_DEBUG("[BLE-TX] ConnectionHandle=%u AttributeHandle=0x%04X NegotiatedAttMtu=%u MaximumNotificationValueLength=%u RequestedValueLength=%u NotificationEnabled=true StreamingEnabled=true Packet=%s ManualLabelRaw=%u",
                      conn_id,
                      HDLC_IAQ_SENSOR_DATA_VALUE,
                      APP_BLE_IAQ_LOCAL_ATT_MTU,
                      APP_BLE_IAQ_NOTIFY_VALUE_MAX,
                      APP_BLE_IAQ_PACKET_LEN,
                      packet_hex,
                      packet[APP_BLE_IAQ_MANUAL_LABEL_OFFSET]);
        wiced_bt_gatt_status_t status = wiced_bt_gatt_server_send_notification(conn_id,
                                                                               HDLC_IAQ_SENSOR_DATA_VALUE,
                                                                               APP_BLE_IAQ_PACKET_LEN,
                                                                               app_iaq_sensor_data,
                                                                               NULL);
        APP_BLE_DEBUG("notify sensor len=%u raw status=%u symbolic status=%s",
                      APP_BLE_IAQ_PACKET_LEN,
                      status,
                      app_gatt_status_name(status));
    }
    else if (0U == conn_id)
    {
        APP_BLE_DEBUG("[BLE-TX] ConnectionHandle=0 AttributeHandle=0x%04X NegotiatedAttMtu=%u MaximumNotificationValueLength=%u RequestedValueLength=%u NotificationEnabled=%s StreamingEnabled=%s Packet=%s ManualLabelRaw=%u",
                      HDLC_IAQ_SENSOR_DATA_VALUE,
                      APP_BLE_IAQ_LOCAL_ATT_MTU,
                      APP_BLE_IAQ_NOTIFY_VALUE_MAX,
                      APP_BLE_IAQ_PACKET_LEN,
                      (app_iaq_sensor_data_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION) ? "true" : "false",
                      streaming_enabled ? "true" : "false",
                      packet_hex,
                      packet[APP_BLE_IAQ_MANUAL_LABEL_OFFSET]);
    }
    else if (!(app_iaq_sensor_data_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION))
    {
        APP_BLE_DEBUG("[BLE-TX] ConnectionHandle=%u AttributeHandle=0x%04X NegotiatedAttMtu=%u MaximumNotificationValueLength=%u RequestedValueLength=%u NotificationEnabled=false StreamingEnabled=%s Packet=%s ManualLabelRaw=%u",
                      conn_id,
                      HDLC_IAQ_SENSOR_DATA_VALUE,
                      APP_BLE_IAQ_LOCAL_ATT_MTU,
                      APP_BLE_IAQ_NOTIFY_VALUE_MAX,
                      APP_BLE_IAQ_PACKET_LEN,
                      streaming_enabled ? "true" : "false",
                      packet_hex,
                      packet[APP_BLE_IAQ_MANUAL_LABEL_OFFSET]);
    }
    else
    {
        APP_BLE_DEBUG("[BLE-TX] ConnectionHandle=%u AttributeHandle=0x%04X NegotiatedAttMtu=%u MaximumNotificationValueLength=%u RequestedValueLength=%u NotificationEnabled=true StreamingEnabled=false Packet=%s ManualLabelRaw=%u",
                      conn_id,
                      HDLC_IAQ_SENSOR_DATA_VALUE,
                      APP_BLE_IAQ_LOCAL_ATT_MTU,
                      APP_BLE_IAQ_NOTIFY_VALUE_MAX,
                      APP_BLE_IAQ_PACKET_LEN,
                      packet_hex,
                      packet[APP_BLE_IAQ_MANUAL_LABEL_OFFSET]);
    }
}
