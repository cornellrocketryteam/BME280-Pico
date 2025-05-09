#include "../bme280.hpp"
#include "tusb.h"
#include <cstdio>

#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

BME280 sensor_1(I2C_PORT);
BME280 sensor_2(I2C_PORT);


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

    while (!sensor_1.begin(0x76)) {
        printf("Error: Sensor failed to initialize\n");
        sleep_ms(1000);
    }

    while (!sensor_2.begin(0x77)) {
        printf("Error: Sensor failed to initialize\n");
        sleep_ms(1000);
    }

    printf("Sensor initialized");

    float temperature_1, temperature_2;

    while (true) {
        if (!sensor_1.read_temperature(&temperature_1)) {
            printf("Error: Sensor failed to read temperature\n");
        }

        if (!sensor_2.read_temperature(&temperature_2)) {
            printf("Error: Sensor failed to read temperature\n");
        }

        printf("Temperature_1: %.2f\n\n", temperature_1);
        printf("Temperature_2: %.2f\n\n", temperature_2);
        sleep_ms(50);
    }

    return 0;
}
