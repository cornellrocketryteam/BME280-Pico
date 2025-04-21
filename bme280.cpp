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
    device.intf_ptr = i2c;
}

bool BME280::begin() {

    int8_t rslt;
    struct bme280_settings settings;
    uint8_t settings_sel;

    // Check Chip ID right away
    uint8_t chip_id = 0;
    rslt = bme280_get_regs(BME280_REG_CHIP_ID, &chip_id, 1, &device);
    if (rslt != BME280_OK || chip_id != BME280_CHIP_ID) {
        printf("Chip ID check failed. Found: 0x%02X (Expected: 0x%02X)\n", chip_id, BME280_CHIP_ID);
        return false;
    }

    rslt = bme280_init(&device);
    printf("Init result: %d\n", rslt);
    if (rslt != BME280_OK) {
        printf("Sensor initialization failed: %d\n", rslt);
        return false;
    }

    rslt = bme280_get_sensor_settings(&settings, &device);
    if (rslt != BME280_OK) {
        printf("Sensor settings retrieval failed: %d\n", rslt);
        return false;
    }
    
    // Apply settings
    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device);
    if (rslt != BME280_OK) {
        printf("Failed to apply settings: %d\n", rslt);
        return false;
    }

    settings_sel = BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP | BME280_SEL_OSR_HUM |
    BME280_SEL_FILTER | BME280_SEL_STANDBY;

    // Force SLEEP mode before setting config
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, &device);
    if (rslt != BME280_OK) {
        printf("Failed to set sleep mode: %d\n", rslt);
        return false;
    }
    device.delay_us(2000, device.intf_ptr);
    
    rslt = bme280_set_sensor_settings(settings_sel, &settings, &device);
    if (rslt != BME280_OK) {
        printf("Failed to apply settings: %d\n", rslt);
        return false;
    }

    // Set NORMAL mode
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device);
    if (rslt != BME280_OK) {
        printf("Failed to set sleep mode: %d\n", rslt);
        return false;
    }
    device.delay_us(2000, device.intf_ptr);

    uint8_t ctrl_meas, ctrl_hum, config;
    bme280_get_regs(BME280_REG_CTRL_MEAS, &ctrl_meas, 1, &device);
    bme280_get_regs(BME280_REG_CTRL_HUM, &ctrl_hum, 1, &device);
    bme280_get_regs(BME280_REG_CONFIG, &config, 1, &device);

    printf("CTRL_MEAS: 0x%02X, CTRL_HUM: 0x%02X, CONFIG: 0x%02X\n", ctrl_meas, ctrl_hum, config);

    // Delay calculation
    rslt = bme280_cal_meas_delay(&period, &settings);
    printf("Measurement delay result: %d\n", rslt);

    printf("\nTemperature calculation (Data displayed are compensated values)\n");
    printf("Measurement time : %lu us\n\n", (long unsigned int)period);

    bme280_get_regs(BME280_REG_CTRL_MEAS, &ctrl_meas, 1, &device);
    printf("CTRL_MEAS: 0x%02X\n", ctrl_meas);

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

        if (!(status_reg & BME280_STATUS_MEAS_DONE))
        {
            /* Measurement time delay given to read sample */
            device.delay_us(period, device.intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data, &device);
            if (rslt != BME280_OK) {
                printf("Sensor read error: %d\n", rslt);
                return false;
            }

#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Temperature[%d]:   %lf ºC\n", idx, comp_data.temperature);
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