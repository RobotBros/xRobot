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


#ifndef _XROBOT_ESP_SPI_H_
#define _XROBOT_ESP_SPI_H_

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/spi_common.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    char *sendBuffer;
    char *recvBuffer;
    size_t sendBufferSize;
    size_t recvBufferSize;
} SPIHandle;

/**
 * @brief Send data to SPI device
 *
 * @param data  The data to be sent
 * @param size  THe size of the data
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spi_send(char *data, size_t size);

/**
 * @brief Initialize the SPI controller as master and 
 *        register SPI device
 *
 * @param sendBuffer    The pointer for SPI send buffer
 * @param recvBuffer    The pointer for SPI recv buffer
 * @return 
 *         - ESP_ERR_INVALID_ARG   if configuration is invalid
 *         - ESP_ERR_INVALID_STATE if host already is in use
 *         - ESP_ERR_NO_MEM        if out of memory
 *         - ESP_OK                on success
 */
esp_err_t spi_init(SPIHandle *handle);

/**
 * @brief Cleanup SPI stuff
 *
 * @return 
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_ERR_INVALID_STATE if device already is freed
 *         - ESP_OK                on success
 */
esp_err_t spi_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
