#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "api_sht35.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "Main";

#define I2C_SDA 5
#define I2C_SCL 4
#define I2C_FREQ 100000  // 100k max=1M
#define SHT35_ADDR 0x44

i2c_port_t I2C_PORT = I2C_NUM_0;

esp_err_t initialize_i2c() {
  ESP_LOGI(TAG, "Initializing I2C");
  i2c_config_t i2c_config;
  i2c_config.sda_io_num = I2C_SDA;
  i2c_config.scl_io_num = I2C_SCL;
  i2c_config.master.clk_speed = I2C_FREQ;
  i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_config.mode = I2C_MODE_MASTER;
  i2c_config.clk_flags = 0;  // if not set some time cause clock errors

  i2c_param_config(I2C_PORT, &i2c_config);
  esp_err_t ret = i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);

  return ret;
}

void start_reading() {
  while (true) {
    if (api_update_sht35() == ESP_OK) {
      printf("Temperature: %.2f, Humidity: %.2f\n", api_sht35_get_temp(),
             api_sht35_get_humi());
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void app_main(void) {
  esp_err_t ret = ESP_OK;
  ret = initialize_i2c();
  vTaskDelay(pdMS_TO_TICKS(500));
  // esp_err_t res;
  //       printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  //       printf("00:         ");
  //       for (uint8_t i = 3; i < 0x78; i++)
  //       {
  //           i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //           i2c_master_start(cmd);
  //           i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
  //           i2c_master_stop(cmd);
    
  //           res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  //           if (i % 16 == 0)
  //               printf("\n%.2x:", i);
  //           if (res == 0)
  //               printf(" %.2x", i);
  //           else
  //               printf(" --");
  //           i2c_cmd_link_delete(cmd);
  //       }
  //       printf("\n\n");
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "I2C initialized successfully");
    api_initialize_sht35(I2C_PORT, SHT35_ADDR, I2C_SCL);
    api_sht35_set_upper_thresholds(34, 32, 78, 73);
    api_sht35_read_upper_thresholds();

    xTaskCreatePinnedToCore(start_reading, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
  } else {
    ESP_LOGE(TAG, "I2C initialized failed, Error: %s", esp_err_to_name(ret));
  }
}