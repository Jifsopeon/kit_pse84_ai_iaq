#include "cycfg_gap.h"

wiced_bt_device_address_t cy_bt_device_address = {0x84, 0xE8, 0x00, 0x50, 0xA0, 0x00};

const uint8_t cy_bt_adv_packet_elem_0[1] = {0x06};
const uint8_t cy_bt_adv_packet_elem_1[9] = {'P', 'S', 'E', '8', '4', '-', 'I', 'A', 'Q'};
const uint8_t cy_bt_adv_packet_elem_2[16] = {__UUID_SERVICE_IAQ};

wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[] =
{
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_FLAG,
        .len = 1,
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_0,
    },
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
        .len = 9,
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_1,
    },
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE,
        .len = 16,
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_2,
    },
};

const uint8_t cy_bt_scan_resp_packet_elem_0[9] = {'P', 'S', 'E', '8', '4', '-', 'I', 'A', 'Q'};

wiced_bt_ble_advert_elem_t cy_bt_scan_resp_packet_data[] =
{
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
        .len = 9,
        .p_data = (uint8_t*)cy_bt_scan_resp_packet_elem_0,
    },
};
