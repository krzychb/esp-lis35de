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

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define TRIGGER_PIN (GPIO_NUM_4)


void blink_task(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void lis35de_task(void *pvParameter)
{
    esp_err_t err;

    gpio_pad_select_gpio(TRIGGER_PIN);
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);

    err = lis35de_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed, err = %d", err);
        while(1);
    }

    while(1) {
        gpio_set_level(TRIGGER_PIN, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(TRIGGER_PIN, 0);

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
    ESP_LOGI(TAG, "Starting tasks");

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&lis35de_task, "lis35de_task", 8 * 1024, NULL, 5, NULL);
}
