/*
 lis35de.c - LIS35DE sensor SPI driver for ESP32

 Copyright (c) 2017 Krzysztof Budzynski <krzychb@gazeta.pl>
 This work is licensed under the Apache License, Version 2.0, January 2004
 See the file LICENSE for details.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "lis35de.h"

static const char *TAG = "LIS35DE Driver";

// SPI Parameters
#define LIS35_QUE_SIZE          10

// LIS35DE Registers
#define LIS35_WRITE             0       // Logical | with register address to write data
#define LIS35_READ              0x80    // Logical | with register address to read data
#define LIS35_ADDR_NO_INC       0       // Logical | with register address to read w/o address incrementing
#define LIS35_ADDR_INC          0x40    // Logical | with register address to read w/  address incrementing

#define LIS35_REG_OUTX          0x29    // OUT_X (29h) register
#define LIS35_REG_OUTY          0x2B    // OUT_Y (2Bh) register
#define LIS35_REG_OUTZ          0x2D    // OUT_Z (2Dh) register

#define LIS35_REG_CR1           0x20    // CTRL_REG1 (20h) register
#define LIS35_REG_CR1_XEN       0x1     // X axis enable. Default value: 1
#define LIS35_REG_CR1_YEN       0x2     // Y axis enable. Default value: 1
#define LIS35_REG_CR1_ZEN       0x4     // Z axis enable. Default value: 1
#define LIS35_REG_CR1_DR_400HZ  0x80    // Data rate selection. Default value: 0
#define LIS35_REG_CR1_ACTIVE    0x40    // Power Down Control. Default value: 0
#define LIS35_REG_CR1_FULL_SCALE 0x20   // Full Scale selection. Default value: 0

#define LIS35_REG_CR2           0x21
#define LIS35_REG_CR2_BOOT      0x40

#define LIS35_CR3               0x22
#define LIS35_CR3_IHL           0x80
#define LIS35_CR3_CLICK_INT     0x7
#define LIS35_CR3_FF1_INT       0x1

#define LIS35_FF_WU_CFG_1       0x30
#define LIS35_FF_WU_SRC_1       0x31
#define LIS35_FF_WU_THS_1       0x32
#define LIS35_FF_WU_DURATION_1  0x33

#define LIS35_CLICK_CFG         0x38
#define LIS35_CLICK_THSY_X      0x3b
#define LIS35_CLICK_THSZ        0x3c
#define LIS35_CLICK_TIME_LIMIT  0x3D

#define LIS35_STATUS_REG        0x27

spi_device_handle_t lis35de_bus;


static esp_err_t lis35de_write(const uint8_t addr, const uint8_t data)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .addr = (uint8_t) addr,
        .length = 1 * 8,    // data length is in bits
        .tx_data[0] = data
    };
    esp_err_t err = spi_device_transmit(lis35de_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send buffer failed, err = %d", err);
    }
    return err;
}


static esp_err_t lis35de_read(const uint8_t addr, uint8_t *data)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .addr = (uint8_t) addr,
        .length = 1 * 8,     // total data length is in bits
        /*
         * in duplex mode the Master
         * is also sending data when receiving
         * make the data send 'all ones'
         */
        .tx_data[0] = 0xff,
        .rxlength  = 1 * 8   // receive data length is in bits
    };
    esp_err_t err = spi_device_transmit(lis35de_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send buffer failed, err = %d", err);
    }
    *data = t.rx_data[0];

    return err;
}


static esp_err_t lis35de_spi_init(lis35de_spi_conf_t *config)
{
    esp_err_t err;
    spi_bus_config_t buscfg = {
        .miso_io_num = config->miso_pin,
        .mosi_io_num = config->mosi_pin,
        .sclk_io_num = config->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,   // 0 means that max transfer size is 4k bytes
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->clk_freq_hz,
        .mode = 0,
        .spics_io_num = config->cs_pin,
        .queue_size = LIS35_QUE_SIZE,
        .address_bits = 8,  // Address size is the same for all transactions
    };
    err = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed, err = %d", err);
    }
    err = spi_bus_add_device(HSPI_HOST, &devcfg, &lis35de_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus device add failed, err = %d", err);
    }
    return err;
}


esp_err_t lis35de_init(lis35de_spi_conf_t *config)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initialize SPI");
    err  = lis35de_spi_init(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed, err = %d", err);
        return err;
    }

    uint8_t addr;
    uint8_t data;

    ESP_LOGI(TAG, "Initialize the LIS35DE sensor");
    addr = LIS35_WRITE | LIS35_ADDR_NO_INC | LIS35_REG_CR2;
    data = LIS35_REG_CR2_BOOT;
    err  = lis35de_write(addr, data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LIS35DE sensor initialization failed, err = %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Power up and enable all three axis");
    addr = LIS35_WRITE | LIS35_ADDR_NO_INC | LIS35_REG_CR1;
    data = LIS35_REG_CR1_XEN | LIS35_REG_CR1_YEN | LIS35_REG_CR1_ZEN | LIS35_REG_CR1_ACTIVE;
    err  = lis35de_write(addr, data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor power up failed, err = %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Check if sensor responds...");
    uint8_t response_check = data;
    addr = LIS35_READ | LIS35_ADDR_NO_INC | LIS35_REG_CR1;
    err  = lis35de_read(addr, &data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initial read failed, err = %d", err);
        return err;
    }
    if (data == response_check) {
        ESP_LOGI(TAG, "Sensor responded correctly");
    } else {
        ESP_LOGE(TAG, "Read from CTRL_REG1 (20h) incorrect : 0x%x, expected 0x%x", data, response_check);
        err = ESP_ERR_LIS35_NOT_DETECTED;
    }
    return err;
}


esp_err_t lis35de_read_position_x(int8_t* x)
{
    uint8_t addr = LIS35_READ | LIS35_ADDR_NO_INC | LIS35_REG_OUTX;
    esp_err_t err  = lis35de_read(addr, (uint8_t *) x);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor x position read failed, err = %d", err);
    }
    return err;
}


esp_err_t lis35de_read_position_y(int8_t* y)
{
    uint8_t addr = LIS35_READ | LIS35_ADDR_NO_INC | LIS35_REG_OUTY;
    esp_err_t err  = lis35de_read(addr, (uint8_t *) y);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor y position read failed, err = %d", err);
    }
    return err;
}


esp_err_t lis35de_read_position_z(int8_t* z)
{
    uint8_t addr = LIS35_READ | LIS35_ADDR_NO_INC | LIS35_REG_OUTZ;
    esp_err_t err  = lis35de_read(addr, (uint8_t *) z);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor z position read failed, err = %d", err);
    }
    return err;
}


esp_err_t lis35de_read_position(int8_t *x, int8_t *y, int8_t *z)
{
    uint8_t data[6];
    spi_transaction_t t = {
        .addr = (uint8_t) LIS35_READ | LIS35_ADDR_INC | LIS35_REG_OUTX,
        .length = 6 * 8,     // total data length is in bits
        .rxlength = 6 * 8,   // receive data length is in bits
        .rx_buffer = data
    };
    esp_err_t err = spi_device_transmit(lis35de_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send buffer failed, err = %d", err);
        return err;
    } else {
        *x = (int8_t) data[0];
        *y = (int8_t) data[2];
        *z = (int8_t) data[4];
        ESP_LOGV(TAG, "[0]:0x%x, [1]:0x%x, [2]:0x%x, [3]:0x%x, [4]:0x%x, [4]:0x%x",
                        data[0],  data[1],  data[2],  data[3],  data[4],  data[4]);
    }
    return err;
}

