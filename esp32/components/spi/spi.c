/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "rom/cache.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "soc/gpio_reg.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "spi.h"


/*
SPI sender (master) example.

This example is supposed to work together with the SPI receiver. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to 
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. The slave makes this pin high as soon as it is
ready to receive/send data. This code connects this line to a GPIO interrupt which gives the rdySem semaphore. The main 
task waits for this semaphore to be given before queueing a transmission.
*/


/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define GPIO_MOSI           25
#define GPIO_MISO           33
#define GPIO_SCLK           26
#define GPIO_CS             27

#define SPI_TAG             "SPI"

static SPIHandle *spi_handle;

// The SPI transaction
static spi_transaction_t tran;

// SPI device handle
static spi_device_handle_t devHandle;

esp_err_t spi_send(char *data, size_t size)
{
    esp_err_t ret;

    if (size > spi_handle->sendBufferSize) {
        ESP_LOGE(SPI_TAG, "Data to send exceed the send buffer size!");
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(spi_handle->sendBuffer, data, size);
    tran.length = size * 8; //size bytes
    ret = spi_device_transmit(devHandle, &tran);
    ESP_LOGD(SPI_TAG, "Received: %s\n", spi_handle->recvBuffer);
    return ret;
}

esp_err_t spi_init(SPIHandle *handle)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=5000000,
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    // Initialize transaction structure
    memset(&tran, 0, sizeof(tran));
    spi_handle = handle;
    tran.tx_buffer = spi_handle->sendBuffer;
    tran.rx_buffer = spi_handle->recvBuffer;

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &devHandle);

    return ret;
}

esp_err_t spi_deinit()
{
    esp_err_t ret;

    ret = spi_bus_remove_device(devHandle);

    return ret;
}
