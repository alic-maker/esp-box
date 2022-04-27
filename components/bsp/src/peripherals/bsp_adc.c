/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/adc.h"

#define TIMES              256
#define GET_UNIT(x)        ((x>>3) & 0x1)

#define ADC_RESULT_BYTE     4
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2

static uint16_t adc1_chan_mask = BIT(3);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[1] = { ADC1_CHANNEL_3};


static uint8_t *g_adc_buf = NULL;
static const char *TAG = "ADC DMA";

static void continuous_adc_init(uint16_t adc1_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 32 * 1024,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = 0,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = false,
        .conv_limit_num = 250,
        .sample_freq_hz = 16 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

esp_err_t bsp_adc_init(void)
{
    esp_err_t ret;

    continuous_adc_init(adc1_chan_mask, channel, 1);
    adc_digi_start();

    g_adc_buf = heap_caps_calloc(1, 16 * 1024, MALLOC_CAP_INTERNAL);
    ESP_RETURN_ON_FALSE(NULL != g_adc_buf, ESP_ERR_NO_MEM, TAG, "no mem for adc buffer");

    return ret;
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2) {
        return false;
    }
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit)) {
        return false;
    }

    return true;
}

extern esp_err_t adc_digi_my_read_bytes(uint8_t *buf, uint32_t length, uint32_t *out_length, TickType_t xTicksToWait);
// esp_err_t adc_digi_my_read_bytes(uint8_t *buf, uint32_t length, uint32_t *out_length, TickType_t xTicksToWait)
// {
//     esp_err_t ret = ESP_OK;
//     uint8_t *data = NULL;
//     size_t size = 0;
//     *out_length = 0;
//     TickType_t xTicksEnd = xTaskGetTickCount() + xTicksToWait;
//     TickType_t xTicksRemaining = xTicksToWait;
//     while (length > 0) {
//         data = xRingbufferReceiveUpTo(s_adc_digi_ctx->ringbuf_hdl, &size, xTicksRemaining, length);
//         if (!data) {
//             ESP_LOGV(ADC_TAG, "No data, increase timeout or reduce conv_num_each_intr");
//             ret = ESP_ERR_TIMEOUT;
//             return ret;
//         }

//         memcpy(buf, data, size);
//         vRingbufferReturnItem(s_adc_digi_ctx->ringbuf_hdl, data);
//         length -= size;
//         assert((size % 4) == 0);
//         *out_length += size;

//         if (xTicksToWait != portMAX_DELAY) {
//             xTicksRemaining = xTicksEnd - xTaskGetTickCount();
//         }
//     }

//     if (s_adc_digi_ctx->ringbuf_overflow_flag) {
//         ret = ESP_ERR_INVALID_STATE;
//     }

//     return ret;
// }

static float last_v = 0;
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

esp_err_t bsp_adc_read(int i2s_num, void *dest, size_t size, size_t *bytes_read, TickType_t ticks_to_wait)
{
    esp_err_t ret;
    uint8_t *result = g_adc_buf;
    int16_t *out = dest;
    uint32_t ret_num = 0;
    ret = adc_digi_my_read_bytes(result, size , &ret_num, ticks_to_wait);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        size_t len = 0;
        for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
            adc_digi_output_data_t *p = (void *)&result[i];

            if (check_valid_data(p)) {
                int16_t d = p->type2.data;
                d -= 2300;
                d <<= 4;
                // float f = d;
                // UTILS_LP_FAST(last_v, f, 0.5);
                // d = (int16_t)last_v;
                
                out[len++] = d;
                out[len++] = d; // copy left channel to right
            } else {
                // abort();
                ESP_LOGI(TAG, "Invalid data [%d_%d_%x]", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
        }
        *bytes_read = 2*len;
        printf("len = %d(%d,%d,%d,%d)\n", *bytes_read, out[0], out[2], out[4], out[6]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        /**
         * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
         * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
         */
        ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
        vTaskDelay(1000);
    }
    return ret;
}

esp_err_t bsp_adc_deinit(void)
{
    esp_err_t ret;
    if (g_adc_buf) {
        heap_caps_free(g_adc_buf);
        g_adc_buf = NULL;
    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    return ret;
}
