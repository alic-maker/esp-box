/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "esp_log.h"
#include "bsp_board.h"
#include "bsp_i2s.h"
#include "bsp_codec.h"
#include "button.h"
#include "bsp_btn.h"


static const board_res_desc_t g_board_rgb_devkit_res = {

    .FUNC_LCD_EN =     (1),
    .LCD_BUS_WIDTH =   (1),
    .LCD_IFACE_SPI =   (0),
    .LCD_DISP_IC_ST =  (1),
    .LCD_WIDTH =       (800),
    .LCD_HEIGHT =      (480),
    .LCD_FREQ =        (13 * 1000 * 1000),
    .LCD_CMD_BITS =    0,
    .LCD_PARAM_BITS =  0,
    .LCD_HOST =        (SPI2_HOST),

    .LCD_SWAP_XY =     (true),
    .LCD_MIRROR_X =    (false),
    .LCD_MIRROR_Y =    (true),
    .LCD_COLOR_INV =   (true),
    .LCD_COLOR_SPACE = ESP_LCD_COLOR_SPACE_RGB,

    .GPIO_LCD_BL =     (GPIO_NUM_NC),
    .GPIO_LCD_BL_ON =  (0),

    // RGB interface GPIOs for LCD panel
    .GPIO_LCD_DISP_EN = GPIO_NUM_NC,
    .GPIO_LCD_VSYNC   = GPIO_NUM_3,
    .GPIO_LCD_HSYNC   = GPIO_NUM_46,
    .GPIO_LCD_DE      = GPIO_NUM_0,
    .GPIO_LCD_PCLK    = GPIO_NUM_9,
    .GPIO_LCD_DATA0   = GPIO_NUM_14, // B0
    .GPIO_LCD_DATA1   = GPIO_NUM_13, // B1
    .GPIO_LCD_DATA2   = GPIO_NUM_12, // B2
    .GPIO_LCD_DATA3   = GPIO_NUM_11, // B3
    .GPIO_LCD_DATA4   = GPIO_NUM_10, // B4
    .GPIO_LCD_DATA5   = GPIO_NUM_39, // G0
    .GPIO_LCD_DATA6   = GPIO_NUM_38, // G1
    .GPIO_LCD_DATA7   = GPIO_NUM_45, // G2
    .GPIO_LCD_DATA8   = GPIO_NUM_48, // G3
    .GPIO_LCD_DATA9   = GPIO_NUM_47, // G4
    .GPIO_LCD_DATA10  = GPIO_NUM_21, // G5
    .GPIO_LCD_DATA11  = GPIO_NUM_1,  // R0
    .GPIO_LCD_DATA12  = GPIO_NUM_2,  // R1
    .GPIO_LCD_DATA13  = GPIO_NUM_42, // R2
    .GPIO_LCD_DATA14  = GPIO_NUM_41, // R3
    .GPIO_LCD_DATA15  = GPIO_NUM_40, // R4

    .BSP_INDEV_IS_TP =         (1),
    .TOUCH_PANEL_SWAP_XY =     (0),
    .TOUCH_PANEL_INVERSE_X =   (0),
    .TOUCH_PANEL_INVERSE_Y =   (0),
    .TOUCH_PANEL_I2C_ADDR = 0,
    .TOUCH_WITH_HOME_BUTTON = 0,

    .BSP_BUTTON_EN =   (0),
    .BUTTON_TAB =  NULL,
    .BUTTON_TAB_LEN = 0,

    .FUNC_I2C_EN =     (1),
    .GPIO_I2C_SCL =    (GPIO_NUM_18),
    .GPIO_I2C_SDA =    (GPIO_NUM_8),

    .FUNC_SDMMC_EN =   (1),
    .SDMMC_BUS_WIDTH = (1),
    .GPIO_SDMMC_CLK =  (GPIO_NUM_6),
    .GPIO_SDMMC_CMD =  (GPIO_NUM_7),
    .GPIO_SDMMC_D0 =   (GPIO_NUM_5),
    .GPIO_SDMMC_D1 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_D2 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_D3 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_DET =  (GPIO_NUM_NC),

    .FUNC_SDSPI_EN =       (0),
    .SDSPI_HOST =          (SPI2_HOST),
    .GPIO_SDSPI_CS =       (GPIO_NUM_NC),
    .GPIO_SDSPI_SCLK =     (GPIO_NUM_NC),
    .GPIO_SDSPI_MISO =     (GPIO_NUM_NC),
    .GPIO_SDSPI_MOSI =     (GPIO_NUM_NC),

    .FUNC_SPI_EN =         (0),
    .GPIO_SPI_CS =         (GPIO_NUM_NC),
    .GPIO_SPI_MISO =       (GPIO_NUM_NC),
    .GPIO_SPI_MOSI =       (GPIO_NUM_NC),
    .GPIO_SPI_SCLK =       (GPIO_NUM_NC),

    .FUNC_RMT_EN =         (0),
    .GPIO_RMT_IR =         (GPIO_NUM_NC),
    .GPIO_RMT_LED =        (GPIO_NUM_NC),

    .FUNC_I2S_EN =         (0),
    .GPIO_I2S_LRCK =       (GPIO_NUM_NC),
    .GPIO_I2S_MCLK =       (GPIO_NUM_NC),
    .GPIO_I2S_SCLK =       (GPIO_NUM_NC),
    .GPIO_I2S_SDIN =       (GPIO_NUM_NC),
    .GPIO_I2S_DOUT =       (GPIO_NUM_NC),
    .CODEC_I2C_ADDR = 0,
    .AUDIO_ADC_I2C_ADDR = 0,

    .IMU_I2C_ADDR = 0,

    .FUNC_PWR_CTRL =       (0),
    .GPIO_PWR_CTRL =       (GPIO_NUM_NC),
    .GPIO_PWR_ON_LEVEL =   (1),

    .GPIO_MUTE_NUM =   GPIO_NUM_NC,
    .GPIO_MUTE_LEVEL = 1,

    .PMOD1 = NULL,
    .PMOD2 = NULL,
};

static const char *TAG = "board";

esp_err_t bsp_board_rgb_devkit_init(void)
{
    ESP_ERROR_CHECK(bsp_codec_init(AUDIO_HAL_16K_SAMPLES));
    return ESP_OK;
}

esp_err_t bsp_board_rgb_devkit_power_ctrl(power_module_t module, bool on)
{
    return ESP_OK;
}

const board_res_desc_t *bsp_board_rgb_devkit_get_res_desc(void)
{
    return &g_board_rgb_devkit_res;
}
