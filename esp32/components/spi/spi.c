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
#define GPIO_RECV_INTR      12

#define SPI_TAG             "SPI"
#define RX_BUF_LENGTH       20     // Length in bytes
#define STACK_SIZE          2048

// Prototype
void spi_recv_task(void *pvParameters);

// SPI Handler
static SPIHandle *spi_handle;

// The SPI transaction
static spi_transaction_t tran;

// SPI device handle
static spi_device_handle_t devHandle;

// SPI Recv task handle
static TaskHandle_t spiRecvTask = NULL;

//The semaphore indicating the slave is ready to receive stuff.
static SemaphoreHandle_t rdySem = NULL;

/*
This ISR is called when the receive line goes low.
*/
static void IRAM_ATTR gpio_recv_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime;
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lasthandshaketime;
    if (diff<240000) return; //ignore everything <1ms after an earlier irq
    lasthandshaketime=currtime;

    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

esp_err_t spi_send(uint8_t *data, size_t size, size_t rxlength)
{
    esp_err_t ret;

    if (size > spi_handle->sendBufferSize) {
        ESP_LOGE(SPI_TAG, "Data to send exceed the send buffer size!");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(spi_handle->sendBuffer, data, size);
    tran.length = size * 8;     //size bytes
    tran.rxlength = rxlength;   // Rx data len
    // NULL if no data recv stage needed
    tran.rx_buffer = rxlength == 0 ? NULL : spi_handle->recvBuffer; 
    ret = spi_device_transmit(devHandle, &tran);
    ESP_LOGD(SPI_TAG, "SPI_SEND: ");
    esp_log_buffer_hex(SPI_TAG, data, size);
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
        .clock_speed_hz=100000,
        .duty_cycle_pos=128,           // 50% duty cycle
        .mode=3,                       // CPOL=1, CPHA=1
        .spics_io_num=-1,
        .flags=SPI_DEVICE_HALFDUPLEX,  // Half duplex
        .cs_ena_posttrans=3,           // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    // Initialize transaction structure
    memset(&tran, 0, sizeof(tran));
    spi_handle = handle;
    tran.tx_buffer = spi_handle->sendBuffer;
    tran.rx_buffer = spi_handle->recvBuffer;

    //Create the semaphore.
    rdySem = xSemaphoreCreateBinary();
    if (rdySem == NULL) {
        ESP_LOGE(SPI_TAG, "Failed to allocate semaphore");
        return ESP_FAIL;
    }

    //GPIO config for the recv interrupt line.
    gpio_config_t io_conf = {
        .intr_type=GPIO_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE,
        .pin_bit_mask=(1<<GPIO_RECV_INTR)
    };

    //Set up receive line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_RECV_INTR, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(GPIO_RECV_INTR, gpio_recv_isr_handler, NULL);

    BaseType_t xReturn = xTaskCreate(
        spi_recv_task,
        "SPI_Recv_TASK",
        STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 3,
        &spiRecvTask
    );
    if (xReturn != pdPASS) {
        ESP_LOGE(SPI_TAG, "Failed to create spi recv task.");
        return ESP_FAIL;
    }
    ESP_LOGD(SPI_TAG, "Recv task created");

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &devHandle);

    ESP_LOGD(SPI_TAG, "SPI init ret: %d", ret);

    return ret;
}

void spi_recv_task(void *pvParameters) {
    esp_err_t ret;
    ESP_LOGD(SPI_TAG, "SPI RECV TASK STARTED");

    while (true) {
        ESP_LOGD(SPI_TAG, "Waiting for recv interrupt...");
        // Wait for recv ready semaphore
        if(xSemaphoreTake(rdySem, portMAX_DELAY) == pdTRUE) {
            tran.length = 0;                        // no need to send
            tran.tx_buffer = NULL;                  // No need to send
            tran.rxlength = RX_BUF_LENGTH * 8;      // Rx data len
            tran.rx_buffer = spi_handle->recvBuffer;
            gpio_set_level(GPIO_CS, 0);
            ret = spi_device_transmit(devHandle, &tran);
            gpio_set_level(GPIO_CS, 1);
            ESP_LOGD(SPI_TAG, "Receive ret: %d, recv buf:", ret);
            esp_log_buffer_hex(SPI_TAG, spi_handle->recvBuffer, RX_BUF_LENGTH);

            // invoke recv callback
            spi_handle->recvCallback(tran.rxlength);
        }
    }
}

esp_err_t spi_deinit()
{
    esp_err_t ret;

    ret = spi_bus_remove_device(devHandle);
    vTaskDelete(&spiRecvTask);

    return ret;
}
