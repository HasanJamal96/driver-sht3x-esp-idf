# driver-sht3x-esp-idf

This repository provides a driver for the SHT3x series of temperature and humidity sensors (SHT30, SHT31, SHT35) using the ESP-IDF framework.

## Features

- Supports single shot and periodic measurement modes.
- Soft reset and status register operations.
- Heater control and alert threshold configuration.
- CRC check for data integrity.
- Easy-to-use API.

## Supported Commands

The driver supports all major commands from the SHT3x datasheet, including:

- Single shot measurements (with/without clock stretching).
- Periodic measurements (0.5Hz, 1Hz, 2Hz, 4Hz, 10Hz).
- Fetching measurement data.
- Serial number readout.
- Heater enable/disable.
- Alert threshold configuration.

## Example Usage

```c
#include "drv_sht32.h"

void app_main() {
    // Initialize the sensor
    sht35_init(I2C_NUM_0, 0x44, 22);

    // Perform a single shot measurement
    float temp = 0, humi = 0;
    if (sht35_read_single_shot_data(SHT35_SINGLE_HIGH_REP_WITHOUT_CLK_STRETCH, &temp, &humi) == ESP_OK) {
        printf("Temperature: %.2f C, Humidity: %.2f %%RH\n", temp, humi);
    }
}
```

## API Reference

See `drv_sht32.h` for the full API:

- `sht35_init()`
- `sht35_reset()`
- `sht35_read_single_shot_data()`
- `sht35_start_periodic_mode()`
- `sht35_stop_periodic_mode()`
- `sht35_fetch_periodic_data()`
- `sht35_update_status_data()`
- `sht35_update_heater()`
- `sht35_write_set_high_threshold()` and related threshold functions


## Author

Your Name - [Your GitHub Profile](https://github.com/yourusername)

