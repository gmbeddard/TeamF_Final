#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO    22        // GPIO pin for I2C master clock
#define I2C_MASTER_SDA_IO    21        // GPIO pin for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ   100000    // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0  // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0  // I2C master do not need buffer

#define TMP102_SENSOR_ADDR   0x48      // TMP102 I2C address
#define TMP102_REG_TEMP      0x00      // Temperature register

static esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
}

static float read_temperature() {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TMP102_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, TMP102_REG_TEMP, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TMP102_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    int16_t raw_temperature = ((int16_t)data[0] << 4) | (data[1] >> 4);
    if (raw_temperature & 0x800) {
        raw_temperature |= ~((1 << 12) - 1); // Sign extend if negative
    }
    return raw_temperature * 0.0625; // Convert to Celsius
}

void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C initialized successfully\n");

    while (1) {
        float temperature = read_temperature();
        printf("Temperature: %.2f°C\n", temperature);
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}
