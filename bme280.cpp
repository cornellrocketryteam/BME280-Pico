#include "bme280.hpp"
#include <cstdio>
#include <cstring>

uint32_t period;

BME280::BME280(i2c_inst_t *i2c_type){
    i2c = i2c_type;

    device.intf = BME280_I2C_INTF;
    device.read = &i2c_read;
    device.write = &i2c_write;
    device.delay_us = &delay_us;
    device.intf_ptr = &i2c;
}

bool BME280::begin() {

    int8_t rslt;
    struct bme280_settings settings;

    rslt = bme280_init(&device);
    rslt = bme280_get_sensor_settings(&settings, &device);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);

    printf("\nTemperature calculation (Data displayed are compensated values)\n");
    printf("Measurement time : %lu us\n\n", (long unsigned int)period);

    return true;
}

bool BME280::read_temperature(float* temperature){
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &device);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            device.delay_us(period, device.intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data, &device);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Temperature[%d]:   %lf deg C\n", idx, comp_data.temperature);
#else
            printf("Temperature[%d]:   %ld deg C\n", idx, (long int)comp_data.temperature);
#endif
            idx++;
        }
    }
    *temperature = comp_data.temperature;
    return true;
}

BME280_INTF_RET_TYPE BME280::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_inst_t *i2c = static_cast<i2c_inst_t *>(intf_ptr);

    if (i2c_write_timeout_us(i2c, BME280_ADDR, &reg_addr, 1, true, BME280_BYTE_TIMEOUT_US) < 1) {
        return 1;
    }
    if (i2c_read_timeout_us(i2c, BME280_ADDR, reg_data, len, false, len * BME280_BYTE_TIMEOUT_US) < 1) {
        return 1;
    }
    return BME280_INTF_RET_SUCCESS;
}

BME280_INTF_RET_TYPE BME280::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_inst_t *i2c = static_cast<i2c_inst_t *>(intf_ptr);

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    if (i2c_write_timeout_us(i2c, BME280_ADDR, buf, len + 1, false, (len + 1) * BME280_BYTE_TIMEOUT_US) < 1) {
        return 1;
    }
    return BME280_INTF_RET_SUCCESS;
}

void BME280::delay_us(uint32_t us, void *intf_ptr) {
    sleep_us(us);
}