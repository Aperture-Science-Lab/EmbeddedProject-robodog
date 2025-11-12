#ifndef LSM6DSOX_DRIVER_H
#define LSM6DSOX_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

// LSM6DSOX Register Definitions
#define LSM6DSOX_ADDR 0x6A
#define CTRL1_XL 0x10      // Accelerometer control
#define CTRL2_G 0x11       // Gyroscope control
#define OUTX_L_XL 0x28     // Accel X low byte
#define OUTX_H_XL 0x29     // Accel X high byte
#define WHO_AM_I 0x0F      // Device ID register

class LSM6DSOX_Driver {
private:
    float accelScale;
    float gyroScale;
    
    // I2C helper functions
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readMultipleRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    
public:
    struct SensorData {
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;
        float temperature;
    };
    
    // Driver lifecycle
    bool begin();
    void end();
    
    // Sensor operations
    bool isDataAvailable();
    bool readSensorData(SensorData &data);
    
    // Configuration
    void setAccelScale(uint8_t scale);    // ±2g, ±4g, ±8g, ±16g
    void setGyroScale(uint8_t scale);     // ±125, ±250, ±500, ±1000, ±2000 dps
    void setOutputDataRate(uint8_t rate); // 12.5Hz to 6.66kHz
    
    // Calibration
    void calibrateAccelerometer();
    void calibrateGyroscope();
};

#endif
