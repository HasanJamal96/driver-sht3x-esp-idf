#ifndef _API_SHT35_H_
#define _API_SHT35_H_


#include "drv_sht35.h"
#include "esp_log.h"
#include "esp_err.h"
#include <sys/time.h>


float api_sht35_temp;
float api_sht35_humi;
bool api_sht35_init = false;
uint32_t api_sht35_status_read_at;
const uint16_t API_SHT35_STATUS_READ_INTERVAL = 5; // time in seconds



esp_err_t api_initialize_sht35(i2c_port_t port, uint8_t addr, uint8_t scl_pin);
esp_err_t api_update_sht35();
float api_sht35_get_temp();
float api_sht35_get_humi();


/**
 * @brief Set Alarm upper threshold values
 * @note Example if upper values set like this
 *          set_temp = 60
 *          clear_temp = 59
 *          set_humi = 80
 *          clear_humi = 79
 *          so the alarm activates if temperature exceeds from 60 and deactivates when reduce below 59
 *          and the alarm activates if humidity exceeds from 80 and deactivates when reduce below 79 
 * 
 * @param[in] set_temp temperature at which alarm set.
 * @param[in] clear_temp temperature at which alarm clear.
 * @param[in] set_humi humidity at which alarm set.
 * @param[in] clear_humi humidity at which alarm clear.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t api_sht35_set_upper_thresholds(int8_t set_temp, int8_t clear_temp, uint8_t set_humi, uint8_t clear_humi) {
    esp_err_t ret = ESP_OK;
    ret = sht35_write_set_high_threshold(set_temp, set_humi);
    if(ret != ESP_OK) return ret;
    ret = sht35_write_clear_high_threshold(clear_temp, clear_humi);
    if(ret == ESP_OK) {
        ESP_LOGI("API SHT35", "Upper threshold set");
    }
    return ret;
}


/**
 * @brief Set Alarm lower threshold values
 * @note Example if upper values set like this
 *          set_temp = -10
 *          clear_temp = -9
 *          set_humi = 20
 *          clear_humi = 21
 *          so the alarm activates if temperature fall below from -9 and deactivates when increase from -10
 *          and the alarm activates if humidity fall below from 20 and deactivates when increase from 21 
 * 
 * @param[in] set_temp temperature at which alarm set.
 * @param[in] clear_temp temperature at which alarm clear.
 * @param[in] set_humi humidity at which alarm set.
 * @param[in] clear_humi humidity at which alarm clear.
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t api_sht35_set_lower_thresholds(int8_t set_temp, int8_t clear_temp, uint8_t set_humi, uint8_t clear_humi) {
    esp_err_t ret = ESP_OK;
    ret = sht35_write_set_low_threshold(set_temp, set_humi);
    if(ret != ESP_OK) return ret;
    return sht35_write_clear_low_threshold(clear_temp, clear_humi);
}

/**
 * @brief Read Alarm upper threshold values
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t api_sht35_read_upper_thresholds() {
    esp_err_t ret = ESP_OK;
    float set_temp;
    float clear_temp;
    float set_humi;
    float clear_humi;
    ret = sht35_read_set_high_threshold(&set_temp, &set_humi);
    if(ret != ESP_OK) return ret;
    ret = sht35_read_clear_high_threshold(&clear_temp, &clear_humi);
    if(ret == ESP_OK) {
        ESP_LOGI("API SHT35", "Upper thresholds: temp: %.2f  %.2f | humi: %.2f  %.2f", set_temp, clear_temp, set_humi, clear_humi);
    }
    return ret;
}


/**
 * @brief Read Alarm lower threshold values
 * 
 * @return esp_err_t ESP_OK on success, error otherwise.
 */
esp_err_t api_sht35_read_lower_thresholds() {
    esp_err_t ret = ESP_OK;
    float set_temp;
    float clear_temp;
    float set_humi;
    float clear_humi;
    ret = sht35_read_set_low_threshold(&set_temp, &set_humi);
    if(ret != ESP_OK) return ret;
    ret = sht35_read_clear_low_threshold(&clear_temp, &clear_humi);
    if(ret == ESP_OK) {
        ESP_LOGI("API SHT35", "Lower thresholds: temp: %.2f  %.2f | humi: %.2f  %.2f", set_temp, clear_temp, set_humi, clear_humi);
    }
    return ret;
}



esp_err_t api_initialize_sht35(i2c_port_t port, uint8_t addr, uint8_t scl_pin) {
    sht35_init(port, addr, scl_pin);
    esp_err_t ret = sht35_reset();
    if (ret != ESP_OK) {
        api_sht35_init = false;
        ESP_LOGE("API SHT35", "Failed to Initialize SHT35\n");
    } else {
        api_sht35_init = true;
        uint32_t s_no = 0;
        ret = sht35_get_serial_number(&s_no);
        if(ret == ESP_OK) {
            ESP_LOGI("API SHT35", "Initialized SHT35 %lu\n", s_no);
            sht35_start_periodic_mode(SHT35_PERIODIC_HIGH_REP_0_5_MPS); // set periodic mode speed
            ret = sht35_clear_alerts();
        }
    }
    return ret;
}

void api_sht35_status_update_loop() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    if(tv.tv_sec - api_sht35_status_read_at >= API_SHT35_STATUS_READ_INTERVAL) {
        esp_err_t ret = sht35_update_status_data();
        if(ret == ESP_OK) {
            api_sht35_status_read_at = tv.tv_sec;
        }
    }
}


esp_err_t api_update_sht35() {
    if(!api_sht35_init) return ESP_ERR_NOT_ALLOWED;
    api_sht35_status_update_loop();
    if(sht35_temp_alert_status()) {
        ESP_LOGI("API SHT35", "Temperature alert...");
    }
    if(sht35_humi_alert_status()) {
        ESP_LOGI("API SHT35", "Humidity alert...");
    }
    // return sht35_fetch_periodic_data(&api_sht35_temp, &api_sht35_humi);
    // return sht35_read_single_shot_data(SHT35_SINGLE_HIGH_REP_WITHOUT_CLK_STRETCH, &api_sht35_temp, &api_sht35_humi);
    return ESP_OK;
}


float api_sht35_get_temp() {
    return api_sht35_temp;
}

float api_sht35_get_humi() {
    return api_sht35_humi;
}


#endif // _API_SHT35_H_