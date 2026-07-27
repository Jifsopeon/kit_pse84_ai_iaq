#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS                         0x1800
#define __UUID_CHARACTERISTIC_DEVICE_NAME                     0x2A00
#define __UUID_CHARACTERISTIC_APPEARANCE                      0x2A01
#define __UUID_SERVICE_GENERIC_ATTRIBUTE                      0x1801
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902

#define __UUID_SERVICE_IAQ                                    0x5B, 0x19, 0xBA, 0xE4, 0xE4, 0x52, 0xA9, 0x96, 0xF1, 0x4A, 0x84, 0xC8, 0x09, 0x4D, 0xC0, 0x21
#define __UUID_CHARACTERISTIC_IAQ_SENSOR_DATA                 0x84, 0x97, 0xCA, 0x92, 0x5F, 0x02, 0x1E, 0xB9, 0x3D, 0x4A, 0x31, 0x6B, 0x43, 0x00, 0x50, 0x1E
#define __UUID_CHARACTERISTIC_IAQ_COMMAND                     0x85, 0x97, 0xCA, 0x92, 0x5F, 0x02, 0x1E, 0xB9, 0x3D, 0x4A, 0x31, 0x6B, 0x43, 0x00, 0x50, 0x1E

#define HDLS_GAP                                              0x0001
#define HDLC_GAP_DEVICE_NAME                                  0x0002
#define HDLC_GAP_DEVICE_NAME_VALUE                            0x0003
#define MAX_LEN_GAP_DEVICE_NAME                               0x0009
#define HDLC_GAP_APPEARANCE                                   0x0004
#define HDLC_GAP_APPEARANCE_VALUE                             0x0005
#define MAX_LEN_GAP_APPEARANCE                                0x0002

#define HDLS_GATT                                             0x0006

#define HDLS_IAQ_SERVICE                                      0x0007
#define HDLC_IAQ_SENSOR_DATA                                  0x0008
#define HDLC_IAQ_SENSOR_DATA_VALUE                            0x0009
#define MAX_LEN_IAQ_SENSOR_DATA                               0x0017
#define HDLD_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG               0x000A
#define MAX_LEN_IAQ_SENSOR_DATA_CLIENT_CHAR_CONFIG            0x0002

#define HDLC_IAQ_COMMAND                                      0x000B
#define HDLC_IAQ_COMMAND_VALUE                                0x000C
#define MAX_LEN_IAQ_COMMAND                                   0x0008

typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t *p_data;
} gatt_db_lookup_table_t;

extern const uint8_t gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern uint8_t app_gap_appearance[];
extern uint8_t app_iaq_sensor_data[];
extern uint8_t app_iaq_sensor_data_client_char_config[];
extern uint8_t app_iaq_command[];

#endif
