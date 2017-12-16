/* LIS35DE Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lis35de.h"

static const char *TAG = "LIS35DE Example";

// SPI Pin and parameters definition
#define CS_PIN          (GPIO_NUM_22)
#define SCK_PIN         (GPIO_NUM_18)
#define MOSI_PIN        (GPIO_NUM_23)
#define MISO_PIN        (GPIO_NUM_19)
#define SPI_FREQ_HZ     8*1000*1000  // Max 10 MHz for LIS35DE


void lis35de_task(void *pvParameter)
{
    esp_err_t err;

    lis35de_spi_conf_t spi_config = {
            .cs_pin = CS_PIN,
            .sck_pin = SCK_PIN,
            .mosi_pin = MOSI_PIN,
            .miso_pin = MISO_PIN,
            .clk_freq_hz = SPI_FREQ_HZ
    };

    err = lis35de_init(&spi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed, err = %d", err);
        while(1);
    }

    while(1) {
        int8_t x, y, z;
        err  = lis35de_read_position(&x, &y, &z);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Sensor position read failed, err = %d", err);
        }
        ESP_LOGI(TAG, "x: %3d, y: %3d, z: %3d", x, y, z);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    ESP_LOGI(TAG, "Starting example");
    xTaskCreate(&lis35de_task, "lis35de_task", 4 * 1024, NULL, 5, NULL);
}
