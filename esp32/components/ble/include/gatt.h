// Copyright 2010-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef _XROBOT_ESP_BLE_H_
#define _XROBOT_ESP_BLE_H_

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/spi_common.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef void (* gatt_recv_cb_t)(uint8_t *data, size_t size);

typedef struct {
    gatt_recv_cb_t  recvCallback;
    // Connection related
    bool            connected;
    esp_gatt_if_t   gatts_if;
    uint16_t        conn_idl;
    uint16_t        noti_attr_handle;
} BLEHandle;

/**
 * @brief Send data to SPI device
 *
 * @param data  The data to be sent
 * @param size  THe size of the data
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t ble_gatt_init(BLEHandle *handle);

/**
 * @brief Send notification to gatt client
 * 
 * @param data The address for the data to sent
 * @param size The size of the data
 * @return
 *         - ESP_FAILED   if send ok
 *         - ESP_OK       on success
 */
esp_err_t ble_send_notification(BLEHandle *handle, uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif
