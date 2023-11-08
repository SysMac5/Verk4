/*
 *  Title:
 *  Description:
 *  Author: 
 */

#pragma once
#include <hardware/i2c.h>

#define BME280_DEFAULT_I2CADDR 0x76 // SDO connected to GND
#define BME280_ALTERNATE_I2CADDR 0x77 // SDO connected to VDDIO
#define BME280_CHIP_ID 0x60

// Memory map for entire documented memory apart from the calibration data.
namespace BME280_REG
{
    const uint8_t HUM_LSB = 0xFE;
    const uint8_t HUM_MSB = 0xFD;
    const uint8_t TEMP_XLSB = 0xFC;
    const uint8_t TEMP_LSB = 0xFB;
    const uint8_t TEMP_MSB = 0xFA;
    const uint8_t PRESS_XLSB = 0xF9;
    const uint8_t PRESS_LSB = 0xF8;
    const uint8_t PRESS_MSB = 0xF7;
    const uint8_t CONFIG = 0xF5;
    const uint8_t CTRL_MEAS = 0xF4;
    const uint8_t STATUS = 0xF3;
    const uint8_t CTRL_HUM = 0xF2;
    const uint8_t RESET = 0xE0;
    const uint8_t ID = 0xD0;
    const uint8_t CAL_FIRST = 0x88;
    const uint8_t CAL_SECOND = 0xE1;
}

enum class BME280_OSR_t : uint8_t {
    SKIP = 0x00,
    X1 = 0x01,
    X2 = 0x02,
    X4 = 0x04,
    X8 = 0x05,
    X16 = 0x06
};

class BME280 {
public:
    /// @brief Construct a new BME280 object
    /// @param i2c Pointer to the I2C hardware instance to be used
    /// @param addr The I2C address for the sensor, default is 0x76 (SDO connected to GND)
    BME280(i2c_inst_t* i2c, uint8_t addr = BME280_DEFAULT_I2CADDR) : _i2c(i2c), _address(addr) {};
    BME280() {}; // Default constructor

    bool init(void);
    void reset(void);
    bool checkConnected(void);
    bool setModeSleep(void);
    bool setModeForced(void);
    bool setModeNormal(void);
    bool setPressureOSR(BME280_OSR_t osr);
    bool setTemperatureOSR(BME280_OSR_t osr);
    bool setHumidityOSR(BME280_OSR_t osr);

    typedef struct __attribute__((__packed__)) {
        float humidity; // Relative humidity in %
        float pressure; // Pressure in hPa
        float temperature; // Temperature in °C
    } BME_Measurement_t;

    bool read(BME_Measurement_t* measurement);
private:
    i2c_inst_t* _i2c;
    uint8_t _address;

    struct BME_Comp_Coeff_t {
        uint16_t dig_T1, dig_P1;
        int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
        uint8_t dig_H1, dig_H3;
        int8_t dig_H6;  
    } comp_coeffs;

    void compensateValues(  float* temperature,
                            float* pressure,
                            float* humidity,
                            int32_t raw_temperature,
                            int32_t raw_pressure,
                            int32_t raw_humidity);

    bool fetchCompensationData(void);
    bool writeRegister(uint8_t reg_address, uint8_t command);
    bool readRegister(uint8_t* data, uint8_t reg_starting_address, uint8_t length = 1);
};