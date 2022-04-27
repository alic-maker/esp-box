/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_adc_init(void);
esp_err_t bsp_adc_read(int i2s_num, void *dest, size_t size, size_t *bytes_read, TickType_t ticks_to_wait);
esp_err_t bsp_adc_deinit(void);

#ifdef __cplusplus
}
#endif
