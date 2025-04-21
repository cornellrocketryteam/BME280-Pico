#ifndef BME280_HPP
#define BME380_HPP

#include "bme280/bme280.h"
#include "bme280/bme280_defs.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BME280_ADDR (0x76)
#define BME280_DEVICE_ID (0x60)
#define BME280_BYTE_TIMEOUT_US (1500)
#define SAMPLE_COUNT  UINT8_C(50)

/**
 * Representation of the BME280 sensor.
 */
class BME280 {
public:
    /**
     * Initializes a BME280 object on an I2C bus.
     * @param i2c_type The I2C bus that the sensor is on
     */
    BME280(i2c_inst_t *i2c_type);

    /**
     * 
     */
    bool begin();  

    /**
     * Reads a temperaure value in degrees Celsius at 0.01
     * resolution, such that a return value 5321 is 53.21 ºC.
     * @param temperature The resulting temperature
     */
    bool read_temperature(float *temperature);

    /**
     * Reads pressure, temperature, and humidity data.
     * @param pressure The resulting pressure
     * @param temperature The resulting temperature
     * @param humidity The resulting humidity
     */
    // bool read_data(float *pressure, float *temperature, float *humidity)

    /**
     * Triggers a software reset of the BME280.
     */
    // bool reset();

private:
    /**
     * Reads the electronic ID of the BME280.
     * @param buf The resulting electronic ID 
     */
    bool get_id(uint8_t *buf);

    /**
     * Return value for I2C reads and writes.
     */
    int ret;

    /**
     * The I2C bus.
     */
    i2c_inst_t *i2c;
};

#endif // BME280_HPP
