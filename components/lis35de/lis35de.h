/*
 lis35de.h - LIS35DE sensor SPI driver for ESP32

 Copyright (c) 2017 Krzysztof Budzynski <krzychb@gazeta.pl>
 This work is licensed under the Apache License, Version 2.0, January 2004
 See the file LICENSE for details.
*/

#ifndef LIS35DE_H
#define LIS35DE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// LIS35DE Status and error codes
#define ESP_ERR_LIS35_BASE                  0x40000
#define ESP_ERR_LIS35_NOT_DETECTED          (ESP_ERR_LIS35_BASE + 1)

esp_err_t lis35de_init();
esp_err_t lis35de_read_position_x(int8_t* x);
esp_err_t lis35de_read_position_y(int8_t* y);
esp_err_t lis35de_read_position_z(int8_t* z);
esp_err_t lis35de_read_position(int8_t *x, int8_t *y, int8_t *z);

#ifdef __cplusplus
}
#endif

#endif  // LIS35DE_H
