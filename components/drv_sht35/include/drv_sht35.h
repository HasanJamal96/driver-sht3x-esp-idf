#ifndef _DRV_SHT32_H_
#define _DRV_SHT32_H_

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"


extern i2c_port_t sht35_i2c_port;
extern uint8_t sht35_i2c_address;

#define WRITE_BIT    I2C_MASTER_WRITE
#define READ_BIT     I2C_MASTER_READ
#define ACK_CHECK_EN 0x01 

#define SHT35_GET_SERIAL_CMD            0x3780
#define SHT35_SOFT_RESET_CMD            0x30A2 // also called re-init in datasheet
#define SHT35_FETCH_DATA_CMD            0xE000
#define SHT35_STOP_PERIODIC_CMD         0x3093 // also called Break command
#define STH35_STATUS_READ_CMD           0xF32D
#define STH35_STATUS_CLEAR_CMD          0x3041
#define STH35_HEATER_ENABLE_CMD         0x306D
#define STH35_HEATER_DISABLE_CMD        0x3066

#define STH35_WRITE_SET_HIGH_THRESHOLD_CMD   0x611D // write upper threshold at which alert activate
#define STH35_WRITE_CLEAR_HIGH_THRESHOLD_CMD 0x6116 // write upper threshold at which alert deactivate
#define STH35_WRITE_SET_LOW_THRESHOLD_CMD   0x6100 // write lower threshold at which alert activate
#define STH35_WRITE_CLEAR_LOW_THRESHOLD_CMD 0x610B // write lower threshold at which alert deactivate

#define STH35_READ_SET_HIGH_THRESHOLD_CMD    0xE11F // read upper threshold at which alert activate
#define STH35_READ_CLEAR_HIGH_THRESHOLD_CMD  0xE114 // read upper threshold at which alert deactivate
#define STH35_READ_SET_LOW_THRESHOLD_CMD    0xE102 // read lower threshold at which alert activate
#define STH35_READ_CLEAR_LOW_THRESHOLD_CMD  0xE109 // read lower threshold at which alert deactivate

/**
 * @brief Single shot measurement modes.
 *  @note In clock stretch Sensor will pull SCL low until measurement is finished
          In No clock stretch sensor will NACK if no data available
 */
typedef enum {
    SHT35_SINGLE_HIGH_REP_WITH_CLK_STRETCH    = 0x2C06,
    SHT35_SINGLE_MED_REP_WITH_CLK_STRETCH     = 0x2C0D,
    SHT35_SINGLE_LOW_REP_WITH_CLK_STRETCH     = 0x2C10,
    SHT35_SINGLE_HIGH_REP_WITHOUT_CLK_STRETCH = 0x2400,
    SHT35_SINGLE_MED_REP_WITHOUT_CLK_STRETCH  = 0x240B,
    SHT35_SINGLE_LOW_REP_WITHOUT_CLK_STRETCH  = 0x2416,
}single_shot_mode_cmd_t;


/**
 * @brief Periodic measurement modes.
 * MPS = Measurement per seconds
 * It is recomended to send Break command before periodic command
 * 
 */
typedef enum {
    SHT35_PERIODIC_HIGH_REP_0_5_MPS = 0x2032,
    SHT35_PERIODIC_MED_REP_0_5_MPS  = 0x2024,
    SHT35_PERIODIC_LOW_REP_0_5_MPS  = 0x202F,

    SHT35_PERIODIC_HIGH_REP_1_MPS   = 0x2130,
    SHT35_PERIODIC_MED_REP_1_MPS    = 0x2126,
    SHT35_PERIODIC_LOW_REP_1_MPS    = 0x212D,

    SHT35_PERIODIC_HIGH_REP_2_MPS   = 0x2236,
    SHT35_PERIODIC_MED_REP_2_MPS    = 0x2220,
    SHT35_PERIODIC_LOW_REP_2_MPS    = 0x222B,

    SHT35_PERIODIC_HIGH_REP_4_MPS  = 0x2334,
    SHT35_PERIODIC_MED_REP_4_MPS   = 0x2322,
    SHT35_PERIODIC_LOW_REP_4_MPS   = 0x2329,

    SHT35_PERIODIC_HIGH_REP_10_MPS  = 0x2737,
    SHT35_PERIODIC_MED_REP_10_MPS   = 0x2721,
    SHT35_PERIODIC_LOW_REP_10_MPS   = 0x272A,
}periodic_mode_cmd_t;


/**
 * @brief Generate CRC for SHT35 data.
 * 
 * @param[in] data Pointer to the data array.
 * @param[in] len Length of the data array.
 * @return Calculated CRC value.
 */
uint8_t sht35_generate_crc(uint8_t *data, uint8_t len);

/**
 * @brief Create and send a 16-bit command to the sensor.
 * 
 * @param[in] command 16-bit command to send.
 */
void sht35_create_command(uint16_t command);

/**
 * @brief Send a raw I2C frame.
 * 
 * @param[in] buffer Pointer to the data buffer.
 * @param[in] length Number of bytes to send.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_send_frame(uint8_t *buffer, uint8_t length);

/**
 * @brief Initialize the SHT35 sensor.
 * 
 * @param[in] port I2C port to use.
 * @param[in] addr I2C address of the sensor.
 * @param[in] scl_pin GPIO pin for SCL.
 */
void sht35_init(i2c_port_t port, uint8_t addr, uint8_t scl_pin);

/**
 * @brief Perform a soft reset.
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_reset();


esp_err_t sht35_get_serial_number(uint32_t *buffer);


/**
 * @brief Start periodic measurement mode.
 * 
 * @param[in] cmd Periodic mode command.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_start_periodic_mode(periodic_mode_cmd_t cmd);

/**
 * @brief Stop periodic measurement mode. Sending this command will
 *        automatically set Single shot mode. This is also called break function/command
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_stop_periodic_mode();

/**
 * @brief Read temperature and humidity in single shot mode.
 * 
 * @param[in] cmd Single shot mode command.
 * @param[out] temp Pointer to store temperature in °C.
 * @param[out] humi Pointer to store humidity in %RH.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_single_shot_data(single_shot_mode_cmd_t cmd, float *temp, float *humi);

/**
 * @brief Fetch temperature and humidity in periodic mode.
 * 
 * @param[out] temp Pointer to store temperature in °C.
 * @param[out] humi Pointer to store humidity in %RH.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_fetch_periodic_data(float *temp, float *humi);

/**
 * @brief Update status register data.
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_update_status_data();

/**
 * @brief Get pending alert status.
 * 
 * @return true if alert is active, false otherwise.
 */
bool sht35_get_pending_alert_status();

/**
 * @brief Get heater status.
 * 
 * @return true if heater is on, false otherwise.
 */
bool sht35_get_heater_on_status();


/**
 * @brief Get humidity alert status.
 * 
 * @return true if humidity alert is active, false otherwise.
 */
bool sht35_humi_alert_status();

/**
 * @brief Get temperature alert status.
 * 
 * @return true if temperature alert is active, false otherwise.
 */
bool sht35_temp_alert_status();

/**
 * @brief Clear alert status register.
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_clear_alerts();


/**
 * @brief Set Heater on and off.
 * 
 * @param[in] enable boolean true-> enable the heater, false disable the heater.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_update_heater(bool enable);


/**
 * @brief Convert threshold values to according to sensor register
 * Convert actual to raw values
 * and use 7 bits MSB of Humidity and 9 bit MSB of temperature
 * combine them 
 * 
 * @param[in] temperature temperature.
 * @param[in] humidity humidity.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
uint16_t sht35_threshold_conersion(int8_t temp, uint8_t humi);


/**
 * @brief Set Alarm Upper High threshold values
 * Upper Alarm activate coditions
 * 
 * @param[in] temperature temperature at which alarm set.
 * @param[in] humidity humidity at which alarm set.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_write_set_high_threshold(int8_t temperature, uint8_t humidity);

/**
 * @brief Set Alarm Upper Low threshold values
 * Upper Alarm deactivate coditions
 * 
 * @param[in] temperature temperature at which alarm set.
 * @param[in] humidity humidity at which alarm set.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_write_clear_high_threshold(int8_t temperature, uint8_t humidity);

/**
 * @brief Set Alarm Lower High threshold values
 * Lower Alarm activate coditions
 * 
 * @param[in] temperature temperature at which alarm set.
 * @param[in] humidity humidity at which alarm set.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_write_set_low_threshold(int8_t temperature, uint8_t humidity);

/**
 * @brief Set Alarm Lower Low threshold values
 * Lower Alarm deactivate coditions
 * 
 * @param[in] temperature temperature at which alarm set.
 * @param[in] humidity humidity at which alarm set.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_write_clear_low_threshold(int8_t temperature, uint8_t humidity);


/**
 * @brief Read Alarm raw values from sensor and convert them.
 * @param[out] temperature float pointer to temperature value.
 * @param[out] humidity float pointer to humidity value.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_raw_and_convert_alert_values(float *temperature, float *humidity);

/**
 * @brief Read Alarm Upper Highs threshold values
 * @param[out] temperature float pointer to temperature value.
 * @param[out] humidity float pointer to humidity value.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_set_high_threshold(float *temperature, float *humidity);

/**
 * @brief Read Alarm Upper Low threshold values
 * @param[out] temperature float pointer to temperature value.
 * @param[out] humidity float pointer to humidity value.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_clear_high_threshold(float *temperature, float *humidity);

/**
 * @brief Read Alarm Lower High threshold values
 * @param[out] temperature float pointer to temperature value.
 * @param[out] humidity float pointer to humidity value.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_set_low_threshold(float *temperature, float *humidity);

/**
 * @brief Read Alarm Lower Low threshold values
 * @param[out] temperature float pointer to temperature value.
 * @param[out] humidity float pointer to humidity value.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t sht35_read_clear_low_threshold(float *temperature, float *humidity);


#endif // _DRV_SHT32_H_