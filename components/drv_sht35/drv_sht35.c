#include "drv_sht35.h"

static const char *TAG = "SHT35 DRV";
static uint8_t communication_buffer[9] = {0};

i2c_port_t sht35_i2c_port;
uint8_t sht35_i2c_address;
uint8_t i2c_scl_pin;

uint16_t sht35_status_register_info;

uint8_t sht35_generate_crc(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void sht35_create_command(uint16_t command) {
  memset(communication_buffer, 0, sizeof(communication_buffer));
  communication_buffer[0] = ((command & 0xFF00) >> 8);
  communication_buffer[1] = (command & 0x00FF);
}

void sht35_init(i2c_port_t port, uint8_t addr, uint8_t scl_pin) {
  sht35_i2c_port = port;
  sht35_i2c_address = addr;
  i2c_scl_pin = scl_pin;
}

esp_err_t sht35_reset() {
  esp_err_t ret = ESP_OK;
  sht35_create_command(
      SHT35_STOP_PERIODIC_CMD);  // reset only works if sensor is in IDLE state
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  sht35_create_command(SHT35_SOFT_RESET_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  vTaskDelay(pdMS_TO_TICKS(1000));
  return ret;
}

esp_err_t sht35_send_frame(uint8_t *buffer, uint8_t length) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (sht35_i2c_address << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  i2c_master_write(cmd, buffer, length, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(sht35_i2c_port, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send i2c command: %s", esp_err_to_name(ret));
    return ret;
  }
  return ret;
}

esp_err_t sht35_receive_frame(uint8_t *buffer) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (sht35_i2c_address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, buffer, 6, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(sht35_i2c_port, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read i2c response: %s", esp_err_to_name(ret));
    return ret;
  }
  return ret;
}

esp_err_t sht35_get_serial_number(uint32_t *buffer) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(SHT35_GET_SERIAL_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  uint8_t rx_buffer[6];
  ret = sht35_receive_frame(rx_buffer);
  if (ret != ESP_OK) return ret;
  uint8_t crc_check_buffer[2];
  for (uint8_t i = 0; i < 5; i += 3) {
    crc_check_buffer[0] = rx_buffer[i];
    crc_check_buffer[1] = rx_buffer[i + 1];
    if (sht35_generate_crc(crc_check_buffer, 2) != rx_buffer[i + 2]) {
      return ESP_ERR_INVALID_CRC;
    }
  }
  *buffer = (rx_buffer[0] << 24) | (rx_buffer[1] << 16) | (rx_buffer[3] << 8) |
            rx_buffer[4];
  return ret;
}

esp_err_t sht35_read_single_shot_data(single_shot_mode_cmd_t cmd, float *temp,
                                      float *humi) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(cmd);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) {
    return ret;
  }
  // in clock stretch mode scl pin will stay low until measurement is completed
  // by sensor
  if (cmd > 0x2C00)
    while (!gpio_get_level(i2c_scl_pin)) {
    }

  uint8_t rx_buffer[9];
  ret = sht35_receive_frame(rx_buffer);
  if (ret != ESP_OK) {
    return ret;
  }
  uint8_t crc_check_buffer[2];
  for (uint8_t i = 0; i < 5; i += 3) {
    crc_check_buffer[0] = rx_buffer[i];
    crc_check_buffer[1] = rx_buffer[i + 1];
    if (sht35_generate_crc(crc_check_buffer, 2) != rx_buffer[i + 2]) {
      return ESP_ERR_INVALID_CRC;
    }
  }
  *temp = rx_buffer[0] << 8 | rx_buffer[1];
  *humi = rx_buffer[3] << 8 | rx_buffer[4];

  *temp = -45.0 + 175.0 * (*temp / 65535.0);
  *humi = 100.0 * (*humi / 65535.0);

  return ret;
}

esp_err_t sht35_start_periodic_mode(periodic_mode_cmd_t cmd) {
  sht35_create_command(cmd);
  return sht35_send_frame(communication_buffer, 2);
}

esp_err_t sht35_stop_periodic_mode() {
  sht35_create_command(SHT35_STOP_PERIODIC_CMD);
  return sht35_send_frame(communication_buffer, 2);
}

esp_err_t sht35_fetch_periodic_data(float *temp, float *humi) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(SHT35_FETCH_DATA_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  uint8_t rx_buffer[9];
  ret = sht35_receive_frame(rx_buffer);
  if (ret != ESP_OK) {
    return ret;
  }
  uint8_t crc_check_buffer[2];
  for (uint8_t i = 0; i < 5; i += 3) {
    crc_check_buffer[0] = rx_buffer[i];
    crc_check_buffer[1] = rx_buffer[i + 1];
    if (sht35_generate_crc(crc_check_buffer, 2) != rx_buffer[i + 2]) {
      return ESP_ERR_INVALID_CRC;
    }
  }
  *temp = rx_buffer[0] << 8 | rx_buffer[1];
  *humi = rx_buffer[3] << 8 | rx_buffer[4];

  *temp = -45.0 + 175.0 * (*temp / 65535.0);
  *humi = 100.0 * (*humi / 65535.0);

  return ret;
}

esp_err_t sht35_update_status_data() {
  esp_err_t ret = ESP_OK;
  sht35_create_command(STH35_STATUS_READ_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;

  uint8_t rx_buffer[3];
  ret = sht35_receive_frame(rx_buffer);
  if (ret != ESP_OK) return ret;
  if (sht35_generate_crc(rx_buffer, 2) != rx_buffer[2]) {
    return ESP_ERR_INVALID_CRC;
  }

  sht35_status_register_info = (rx_buffer[0] << 8) | rx_buffer[1];
  printf("Value in binary: ");
  for (int i = 15; i >= 0; i--) {
    printf("%d", (sht35_status_register_info >> i) & 1);
  }
  printf("\n");
  return ret;
}

esp_err_t sht35_update_heater(bool enable) {
  sht35_create_command(enable ? STH35_HEATER_ENABLE_CMD
                              : STH35_HEATER_DISABLE_CMD);
  return sht35_send_frame(communication_buffer, 2);
}


// Write alert regiser functions

void sht35_create_write_command(uint16_t cmd, uint16_t value) {
  memset(communication_buffer, 0, sizeof(communication_buffer));
  communication_buffer[0] = ((cmd & 0xFF00) >> 8);
  communication_buffer[1] = (cmd & 0x00FF);
  communication_buffer[2] = ((value & 0xFF00) >> 8);
  communication_buffer[3] = (value & 0x00FF);
  uint8_t crc_check_buffer[2];
  crc_check_buffer[0] = communication_buffer[2];
  crc_check_buffer[1] = communication_buffer[3];
  communication_buffer[4] = sht35_generate_crc(crc_check_buffer, 2);
}

uint16_t sht35_threshold_conersion(int8_t temp, uint8_t humi) {
  uint16_t raw_temperature = (((temp + 45) * 65535) / 175);
  uint16_t raw_humidity = ((humi * 65535) / 100);
  raw_humidity = (raw_humidity) & 0xFE00;        // 7 bit MSB
  raw_temperature = (raw_temperature) & 0xFF80;  // 9 bit MSB
  uint16_t threshold_values = raw_humidity | (raw_temperature >> 7);
  return threshold_values;
}

esp_err_t sht35_write_set_high_threshold(int8_t temperature, uint8_t humidity) {
  uint16_t threshold_values = sht35_threshold_conersion(temperature, humidity);
  sht35_create_write_command(STH35_WRITE_SET_HIGH_THRESHOLD_CMD, threshold_values);
  return sht35_send_frame(communication_buffer, 5);
}

esp_err_t sht35_write_clear_high_threshold(int8_t temperature, uint8_t humidity) {
  uint16_t threshold_values = sht35_threshold_conersion(temperature, humidity);
  sht35_create_write_command(STH35_WRITE_CLEAR_HIGH_THRESHOLD_CMD, threshold_values);
  return sht35_send_frame(communication_buffer, 5);
}

esp_err_t sht35_write_set_low_threshold(int8_t temperature, uint8_t humidity) {
  uint16_t threshold_values = sht35_threshold_conersion(temperature, humidity);
  sht35_create_write_command(STH35_WRITE_SET_LOW_THRESHOLD_CMD, threshold_values);
  return sht35_send_frame(communication_buffer, 5);
}

esp_err_t sht35_write_clear_low_threshold(int8_t temperature, uint8_t humidity) {
  uint16_t threshold_values = sht35_threshold_conersion(temperature, humidity);
  sht35_create_write_command(STH35_WRITE_CLEAR_LOW_THRESHOLD_CMD, threshold_values);
  return sht35_send_frame(communication_buffer, 5);
}



// Read alert regiser functions


esp_err_t sht35_read_raw_and_convert_alert_values(float *temperature, float *humidity) {
  esp_err_t ret = ESP_OK;
  uint8_t rx_buffer[9];
  ret = sht35_receive_frame(rx_buffer);
  if (ret != ESP_OK) {
    return ret;
  }
  ESP_LOG_BUFFER_HEX(TAG, rx_buffer, 9);
  uint8_t crc_check_buffer[2];
  crc_check_buffer[0] = rx_buffer[0];
  crc_check_buffer[1] = rx_buffer[1];
  if (sht35_generate_crc(crc_check_buffer, 2) != rx_buffer[2]) {
    return ESP_ERR_INVALID_CRC;
  }

  uint16_t combined_value = (rx_buffer[0] << 8) | rx_buffer[1];
  uint16_t raw_humidity    = combined_value & 0xFE00;
  uint16_t raw_temperature = (combined_value & 0x01FF) << 7;

  *temperature = -45.0 + 175.0 * (raw_temperature / 65535.0);
  *humidity    = 100.0 * (raw_humidity / 65535.0);
  return ret;
}

esp_err_t sht35_read_set_high_threshold(float *temperature, float *humidity) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(STH35_READ_SET_HIGH_THRESHOLD_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  return sht35_read_raw_and_convert_alert_values(temperature, humidity);
}

esp_err_t sht35_read_clear_high_threshold(float *temperature, float *humidity) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(STH35_READ_CLEAR_HIGH_THRESHOLD_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  return sht35_read_raw_and_convert_alert_values(temperature, humidity);
}

esp_err_t sht35_read_set_low_threshold(float *temperature, float *humidity) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(STH35_READ_SET_LOW_THRESHOLD_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  return sht35_read_raw_and_convert_alert_values(temperature, humidity);
}

esp_err_t sht35_read_clear_low_threshold(float *temperature, float *humidity) {
  esp_err_t ret = ESP_OK;
  sht35_create_command(STH35_READ_CLEAR_LOW_THRESHOLD_CMD);
  ret = sht35_send_frame(communication_buffer, 2);
  if (ret != ESP_OK) return ret;
  return sht35_read_raw_and_convert_alert_values(temperature, humidity);
}



// status register functions

bool sht35_get_pending_alert_status() {
  return sht35_status_register_info & 0x8000;  // bit 15
}

bool sht35_get_heater_on_status() {
  return sht35_status_register_info & 0x2000;  // bit 13
}

bool sht35_humi_alert_status() {
  return sht35_status_register_info & 0x0800;  // bit 11
}

bool sht35_temp_alert_status() {
  return sht35_status_register_info & 0x0600;  // bit 11
}

esp_err_t sht35_clear_alerts() {
  sht35_create_command(STH35_STATUS_CLEAR_CMD);
  return sht35_send_frame(communication_buffer, 2);
}