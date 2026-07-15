#if !defined(CYCFG_BT_SETTINGS_H)
#define CYCFG_BT_SETTINGS_H

#include "wiced_bt_cfg.h"
#include "wiced_bt_ble.h"

extern const wiced_bt_cfg_ble_scan_settings_t cy_bt_cfg_scan_settings;
extern const wiced_bt_cfg_ble_advert_settings_t cy_bt_cfg_adv_settings;
extern const wiced_bt_cfg_ble_t cy_bt_cfg_ble;
extern const wiced_bt_cfg_gatt_t cy_bt_cfg_gatt;
extern const wiced_bt_cfg_settings_t cy_bt_cfg_settings;

#define CY_BT_ISOC_MAX_SDU_SIZE          0
#define CY_BT_ISOC_MAX_AUDIO_CHANNELS    0
#define CY_BT_ISOC_MAX_CIS_CONNECTIONS   0
#define CY_BT_ISOC_MAX_CIG_CONNECTIONS   0
#define CY_BT_ISOC_MAX_BUFS_PER_CIS      0
#define CY_BT_ISOC_MAX_BIG_CONNECTIONS   0

#endif
