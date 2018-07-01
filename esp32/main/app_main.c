/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "spi.h"
#include "gatt.h"

#include "sdkconfig.h"

static char spiSendBuffer[CONFIG_SEND_BUFFER_SIZE] = "";
static char spiRecvBuffer[CONFIG_RECV_BUFFER_SIZE] = "";
static SPIHandle spiHandle = {0};

void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize SPI
    spiHandle.sendBuffer = spiSendBuffer;
    spiHandle.recvBuffer = spiRecvBuffer;
    spiHandle.sendBufferSize = CONFIG_SEND_BUFFER_SIZE;
    spiHandle.recvBufferSize = CONFIG_RECV_BUFFER_SIZE;
    ret = spi_init(&spiHandle);
    ESP_ERROR_CHECK(ret);

    // Initialize BLE
    ret = ble_gatt_init();
    ESP_ERROR_CHECK(ret);

    return;
}
