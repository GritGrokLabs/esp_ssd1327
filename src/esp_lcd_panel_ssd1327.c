/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ssd1327.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "lcd_panel.ssd1327";

// SSD1327 Commands
#define SSD1327_CMD_DISPLAY_OFF 0xAE
#define SSD1327_CMD_DISPLAY_ON 0xAF
#define SSD1327_CMD_SET_REMAP 0xA0
#define SSD1327_CMD_SET_CONTRAST 0x81
#define SSD1327_CMD_SET_MUX_RATIO 0xA8
#define SSD1327_CMD_SET_ROW_ADDR 0x75
#define SSD1327_CMD_SET_COL_ADDR 0x15
#define SSD1327_CMD_SET_OFFSET 0xA2
#define SSD1327_CMD_SET_START_LINE 0xA1
#define SSD1327_CMD_SET_PHASE_LEN 0xB1
#define SSD1327_CMD_SET_VCOMH 0xBE
#define SSD1327_CMD_SET_CLOCK 0xB3
#define SSD1327_CMD_ENABLE_VDD 0xAB
#define SSD1327_CMD_SET_PRECHARGE_1 0xBC
#define SSD1327_CMD_SET_PRECHARGE_2 0xB6
#define SSD1327_CMD_FUNC_SEL 0xD5
#define SSD1327_CMD_INVERT_OFF 0xA4
#define SSD1327_CMD_DISPLAY_ALLOFF 0xA6
#define SSD1327_CMD_INVERT_ON 0xA7
#define SSD1327_CMD_SET_LOCK 0xFD

#define SET_REMAP_ENABLE_COL_ADDR_REMAP 0b00000001
#define SET_REMAP_ENABLE_NIBBLE_REMAP 0b00000010
#define SET_REMAP_ENABLE_VERT_ADDR_INCR 0b00000100
#define SET_REMAP_ENABLE_COM_REMAP 0b00010000
#define SET_REMAP_ENABLE_COM_SPLIT 0b01000000

static esp_err_t panel_ssd1327_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1327_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1327_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1327_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1327_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1327_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1327_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1327_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1327_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct
{
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    uint8_t height;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    bool reset_level;
    bool swap_axes;
} ssd1327_panel_t;

esp_err_t esp_lcd_new_panel_ssd1327(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    ssd1327_panel_t *ssd1327 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(panel_dev_config->bits_per_pixel == 4, ESP_ERR_INVALID_ARG, err, TAG, "bpp must be 4");
    esp_lcd_panel_ssd1327_config_t *ssd1327_spec_config = (esp_lcd_panel_ssd1327_config_t *)panel_dev_config->vendor_config;
    ssd1327 = calloc(1, sizeof(ssd1327_panel_t));
    ESP_GOTO_ON_FALSE(ssd1327, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1327 panel");

    if (panel_dev_config->reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    ssd1327->io = io;
    ssd1327->bits_per_pixel = panel_dev_config->bits_per_pixel;
    ssd1327->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1327->reset_level = panel_dev_config->flags.reset_active_high;
    ssd1327->height = ssd1327_spec_config ? ssd1327_spec_config->height : 128;
    ssd1327->base.del = panel_ssd1327_del;
    ssd1327->base.reset = panel_ssd1327_reset;
    ssd1327->base.init = panel_ssd1327_init;
    ssd1327->base.draw_bitmap = panel_ssd1327_draw_bitmap;
    ssd1327->base.invert_color = panel_ssd1327_invert_color;
    ssd1327->base.set_gap = panel_ssd1327_set_gap;
    ssd1327->base.mirror = panel_ssd1327_mirror;
    ssd1327->base.swap_xy = panel_ssd1327_swap_xy;
    ssd1327->base.disp_on_off = panel_ssd1327_disp_on_off;
    *ret_panel = &(ssd1327->base);
    ESP_LOGD(TAG, "new ssd1327 panel @%p", ssd1327);

    return ESP_OK;

err:
    if (ssd1327)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ssd1327);
    }
    return ret;
}

static esp_err_t panel_ssd1327_del(esp_lcd_panel_t *panel)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_del");

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    if (ssd1327->reset_gpio_num >= 0)
    {
        gpio_reset_pin(ssd1327->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ssd1327 panel @%p", ssd1327);
    free(ssd1327);
    return ESP_OK;
}

static esp_err_t panel_ssd1327_reset(esp_lcd_panel_t *panel)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_reset");

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);

    // perform hardware reset
    if (ssd1327->reset_gpio_num >= 0)
    {
        gpio_set_level(ssd1327->reset_gpio_num, ssd1327->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ssd1327->reset_gpio_num, !ssd1327->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t panel_ssd1327_init(esp_lcd_panel_t *panel)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_init");

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    esp_lcd_panel_io_handle_t io_handle = ssd1327->io;

    esp_err_t err = ESP_OK;

    // Unlock the OLED Driver so it can receive commands
    uint8_t cmdclk = 0x12;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_LOCK, &cmdclk, 1);

    // Turn off the display
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_DISPLAY_OFF, NULL, 0);

    // Set contrast (Currently set to max)
    uint8_t contrast = 0xFF; // Adjust for brightness
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_CONTRAST, &contrast, 1);

    // Set remap (horizontal addressing and grayscale mode)
    // The SET_REMAP_ENABLE_COM_SPLIT seems to be required due to the physical wiring of the IC to the OLED Display.
    // The nibble swap is just to get the nibbles to be correct when read left-to-right.
    uint8_t remap = SET_REMAP_ENABLE_COM_SPLIT | SET_REMAP_ENABLE_NIBBLE_REMAP;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_REMAP, &remap, 1);

    // Set display start line (0)
    uint8_t start_line = 0x00;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_START_LINE, &start_line, 1);

    // Set display offset (0)
    uint8_t offset = 0x00;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_OFFSET, &offset, 1);

    // Set multiplex ratio (128 rows)
    uint8_t mux_ratio = 0x7F;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_MUX_RATIO, &mux_ratio, 1);

    // Set phase length
    uint8_t phase_length = 0x11;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_PHASE_LEN, &phase_length, 1);

    // Set clock divider
    uint8_t clock_div = 0x01;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_CLOCK, &clock_div, 1);

    // Enable VDD regulator
    uint8_t enable_vdd = 0x01;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_ENABLE_VDD, &enable_vdd, 1);

    // Set VCOMH voltage
    uint8_t vcomh = 0x0F;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_VCOMH, &vcomh, 1);

    // Enable second pre-charge period
    uint8_t precharge = 0x62;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_FUNC_SEL, &precharge, 1);

    // Set first pre-charge period
    uint8_t precharge1 = 0x08;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_PRECHARGE_1, &precharge1, 1);

    // Set second precharge period
    uint8_t precharge_2 = 0x04;
    err |= esp_lcd_panel_io_tx_param(io_handle, SSD1327_CMD_SET_PRECHARGE_2, &precharge_2, 1);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "SSD1327 initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG, "SSD1327 initialization failed");
    }

    return err;
}

// Function to convert RGB565 to 4-bit grayscale. This is effectively a no-op as the
// LVGL configuration I'm using has 8-bit monochrome configured.
uint8_t rgb565_to_4bit_grayscale(uint16_t rgb565)
{
    return (rgb565 & 0xF);
}

static esp_err_t panel_ssd1327_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_draw_bitmap: start (%d, %d) end (%d, %d)", x_start, y_start, x_end, y_end);

    int pixel_count = (y_end - y_start) * (x_end - x_start);

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1327->io;

    // adding extra gap
    x_start += ssd1327->x_gap;
    x_end += ssd1327->x_gap;
    y_start += ssd1327->y_gap;
    y_end += ssd1327->y_gap;

    if (ssd1327->swap_axes)
    {
        int x = x_start;
        x_start = y_start;
        y_start = x;
        x = x_end;
        x_end = y_end;
        y_end = x;
    }
    // dividing by two in order to get the nibble-based indexes.
    uint8_t col_start = x_start / 2;
    uint8_t col_end = (x_end - 1) / 2;

    ESP_LOGD(TAG, "Page start set:end to %d:%d. Pixel Count %d", col_start, col_end, pixel_count);

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_ROW_ADDR, (uint8_t[]){
                                                                                    (y_start & 0x7F),
                                                                                    ((y_end - 1) & 0x7F),
                                                                                },
                                                  2),
                        TAG, "io tx param SSD1327_CMD_SET_ROW_ADDR failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_COL_ADDR, (uint8_t[]){
                                                                                    (col_start & 0x3F),
                                                                                    (col_end & 0x3F),
                                                                                },
                                                  2),
                        TAG, "io tx param SSD1327_CMD_SET_COL_ADDR failed");

    // transfer frame buffer sizing (taking into account the 4-bit grayscale format)
    size_t frame_buffer_size = pixel_count * (((float)ssd1327->bits_per_pixel) / 8);

    uint8_t *cur_src_pixel = (uint8_t *)color_data;
    uint8_t *cur_dest_pixel = (uint8_t *)color_data;

    // We are going to walk through every array-entry in the color_data array. We are going to do an
    // in-place replacement of the 8-bit grayscale values with 4-bit grayscale values.
    for (int i = 0; i < pixel_count; i++)
    {
        // Convert two 8-bit grayscale pixels to a single 4-bit grayscale byte
        uint8_t pixel1 = *cur_src_pixel >> 4;       // High nibble (quantize 8-bit to 4-bit)
        uint8_t pixel2 = *(cur_src_pixel + 1) >> 4; // Low nibble
        *cur_dest_pixel = (rgb565_to_4bit_grayscale(pixel1) << 4) | rgb565_to_4bit_grayscale(pixel2);

        cur_src_pixel += 2;
        cur_dest_pixel++;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, -1, color_data, frame_buffer_size), TAG, "io tx color failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1327_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_invert_color: %d", invert_color_data);

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1327->io;
    int command = 0;
    if (invert_color_data)
    {
        command = SSD1327_CMD_INVERT_ON;
    }
    else
    {
        command = SSD1327_CMD_INVERT_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1327_CMD_INVERT_ON/OFF failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1327_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_mirror: %d, %d", mirror_x, mirror_y);

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1327->io;

    // Each of these conditions result in a "return", so this might be bad code, but it's safe and simple.
    if (mirror_x && mirror_y)
    {
        // swap x & y
        uint8_t remap = SET_REMAP_ENABLE_COM_SPLIT | SET_REMAP_ENABLE_COM_REMAP | SET_REMAP_ENABLE_COL_ADDR_REMAP;
        return esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_REMAP, &remap, 1);
    }
    if (mirror_x)
    {
        // swap x
        uint8_t remap = SET_REMAP_ENABLE_COM_SPLIT | SET_REMAP_ENABLE_COL_ADDR_REMAP;
        return esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_REMAP, &remap, 1);
    }
    if (mirror_y)
    {
        // swap y
        uint8_t remap = SET_REMAP_ENABLE_COM_SPLIT | SET_REMAP_ENABLE_NIBBLE_REMAP | SET_REMAP_ENABLE_COM_REMAP;
        return esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_REMAP, &remap, 1);
    }

    // no mirroring; reset to default.
    uint8_t remap = SET_REMAP_ENABLE_COM_SPLIT | SET_REMAP_ENABLE_NIBBLE_REMAP;
    return esp_lcd_panel_io_tx_param(io, SSD1327_CMD_SET_REMAP, &remap, 1);
}

static esp_err_t panel_ssd1327_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_swap_xy: %d", swap_axes);
    return ESP_OK;

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    ssd1327->swap_axes = swap_axes;

    return ESP_OK;
}

static esp_err_t panel_ssd1327_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ESP_LOGD(TAG, "Top of panel_ssd1327_set_gap: %d, %d", x_gap, y_gap);

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    ssd1327->x_gap = x_gap;
    ssd1327->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ssd1327_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{

    ESP_LOGD(TAG, "Top of display on/off: %d", on_off);

    ssd1327_panel_t *ssd1327 = __containerof(panel, ssd1327_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1327->io;
    int command = 0;
    if (on_off)
    {
        command = SSD1327_CMD_DISPLAY_ON;
    }
    else
    {
        command = SSD1327_CMD_DISPLAY_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1327_CMD_DISP_ON/OFF failed");

    // SEG/COM will be ON/OFF after 100ms after sending DISP_ON/OFF command
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}
