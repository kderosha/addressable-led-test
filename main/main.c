/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

// GPIO assignment
#define LED_STRIP_GPIO_PIN 13
#define MOTION_SENSOR_INPUT_PIN 27
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 400
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

static const char *TAG = "example";

led_strip_handle_t configure_led(void) {
  // LED strip general initialization, according to your led board design
  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_STRIP_GPIO_PIN,
      .max_leds = LED_STRIP_LED_COUNT,
      .led_model = LED_MODEL_WS2812,
      .color_component_format =
          LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip:
      .flags = {
          .invert_out = false, // don't invert the output signal
      }};

  // LED strip backend configuration: RMT
  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to
                                      // different power consumption
      .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
      .mem_block_symbols =
          64, // the memory size of each RMT channel, in words (4 bytes)
      .flags = {
          .with_dma =
              false, // DMA feature is available on chips like ESP32-S3/P4
      }};

  // LED Strip object handle
  led_strip_handle_t led_strip;
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  ESP_LOGI(TAG, "Created LED strip object with RMT backend");
  return led_strip;
}

int scaleBrightness(int colorToScale) {
  double scaleFactor = .5;
  return colorToScale * scaleFactor;
}

void app_main(void) {
  led_strip_handle_t led_strip = configure_led();
  int counter = 0;
  while (1) {
    if (counter > 3) {
      counter = 0;
    }
    counter++;

    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
      if ((i + counter) % 3 == 0) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 100));
      } else if ((i + counter) % 2 == 0) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 100, 0));
      } else {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 100, 0, 0));
      }
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
