#include "cycfg_gatt_db.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"

const uint8_t gatt_database[] =
{
    PRIMARY_SERVICE_UUID16(HDLS_GAP, __UUID_SERVICE_GENERIC_ACCESS),
        CHARACTERISTIC_UUID16(HDLC_GAP_DEVICE_NAME,
                              HDLC_GAP_DEVICE_NAME_VALUE,
                              __UUID_CHARACTERISTIC_DEVICE_NAME,
                              GATTDB_CHAR_PROP_READ,
                              GATTDB_PERM_READABLE),
        CHARACTERISTIC_UUID16(HDLC_GAP_APPEARANCE,
                              HDLC_GAP_APPEARANCE_VALUE,
                              __UUID_CHARACTERISTIC_APPEARANCE,
                              GATTDB_CHAR_PROP_READ,
                              GATTDB_PERM_READABLE),

    PRIMARY_SERVICE_UUID16(HDLS_GATT, __UUID_SERVICE_GENERIC_ATTRIBUTE),

    PRIMARY_SERVICE_UUID128(HDLS_IAQ_SERVICE, __UUID_SERVICE_IAQ),
        CHARACTERISTIC_UUID128(HDLC_IAQ_SENSOR_DATA,
                               HDLC_IAQ_SENSOR_DATA_VALUE,
                               __UUID_CHARACTERISTIC_IAQ_SENSOR_DATA,
                               GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY,
                               GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_READABLE),
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG,
                                            __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_IAQ_COMMAND,
                                        HDLC_IAQ_COMMAND_VALUE,
                                        __UUID_CHARACTERISTIC_IAQ_COMMAND,
                                        GATTDB_CHAR_PROP_WRITE,
                                        GATTDB_PERM_WRITE_REQ | GATTDB_PERM_WRITE_CMD),
};

const uint16_t gatt_database_len = sizeof(gatt_database);

uint8_t app_gap_device_name[] = {'P', 'S', 'E', '8', '4', '-', 'I', 'A', 'Q', '\0'};
uint8_t app_gap_appearance[] = {0x00, 0x00};
uint8_t app_iaq_sensor_data[MAX_LEN_IAQ_SENSOR_DATA] = {0x00};
uint8_t app_iaq_sensor_data_client_char_config[] = {0x00, 0x00};
uint8_t app_iaq_command[MAX_LEN_IAQ_COMMAND] = {0x00};

gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[] =
{
    {HDLC_GAP_DEVICE_NAME_VALUE, MAX_LEN_GAP_DEVICE_NAME, 9, app_gap_device_name},
    {HDLC_GAP_APPEARANCE_VALUE, MAX_LEN_GAP_APPEARANCE, 2, app_gap_appearance},
    {HDLC_IAQ_SENSOR_DATA_VALUE, MAX_LEN_IAQ_SENSOR_DATA, MAX_LEN_IAQ_SENSOR_DATA, app_iaq_sensor_data},
    {HDLD_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG, MAX_LEN_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG, 2, app_iaq_sensor_data_client_char_config},
    {HDLC_IAQ_COMMAND_VALUE, MAX_LEN_IAQ_COMMAND, 0, app_iaq_command},
};

const uint16_t app_gatt_db_ext_attr_tbl_size =
    (sizeof(app_gatt_db_ext_attr_tbl) / sizeof(gatt_db_lookup_table_t));
