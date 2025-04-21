#include "../bme280.hpp"
#include "tusb.h"
#include <cstdio>

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

BME280 sensor(I2C_PORT);

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    printf("Connected\n");

    while (!sensor.begin()) {
        printf("Error: Sensor failed to initialize\n");
        sleep_ms(1000);
    }

    printf("Sensor initialized");

    float temperature;

    while (true) {
        if (!sensor.read_temperature(&temperature)) {
            printf("Error: Sensor failed to read temperature\n");
        }

        printf("Temperature: %.2f\n\n", temperature);
        sleep_ms(50);
    }

    return 0;
}