#include "LSM6DSOX_Driver.h"

// Read a single register byte
uint8_t LSM6DSOX_Driver::readRegister(uint8_t reg) {
    Wire.beginTransmission(LSM6DSOX_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // Repeated start
    
    Wire.requestFrom(LSM6DSOX_ADDR, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

// Write a single register byte
void LSM6DSOX_Driver::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(LSM6DSOX_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Read multiple consecutive registers
void LSM6DSOX_Driver::readMultipleRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(LSM6DSOX_ADDR);
    Wire.write(reg);  // Auto-increment is enabled by default in LSM6DSOX
    Wire.endTransmission(false);  // Repeated start for read
    
    Wire.requestFrom(LSM6DSOX_ADDR, length);
    for (uint8_t i = 0; i < length; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        } else {
            buffer[i] = 0;
        }
    }
}

// Initialize the sensor
bool LSM6DSOX_Driver::begin() {
    Wire.begin();
    delay(100);
    
    // Verify device communication (WHO_AM_I = 0x6C)
    uint8_t whoAmI = readRegister(WHO_AM_I);
    if (whoAmI != 0x6C) {
        Serial.println("LSM6DSOX not detected!");
        return false;
    }
    
    // Set default scale factors
    accelScale = 0.061f;  // mg per LSB for ±2g range
    gyroScale = 4.375f;   // mdps per LSB for ±125dps range
    
    // Configure accelerometer (104 Hz, ±2g)
    writeRegister(CTRL1_XL, 0x40);
    
    // Configure gyroscope (104 Hz, ±125 dps)
    writeRegister(CTRL2_G, 0x40);
    
    return true;
}

// Read sensor data (accelerometer + gyroscope + temperature)
bool LSM6DSOX_Driver::readSensorData(SensorData &data) {
    // Status register to check data availability
    uint8_t status = readRegister(0x1E);
    if (!(status & 0x01)) return false;  // No new accel data
    
    uint8_t buffer[14];
    readMultipleRegisters(OUTX_L_XL, buffer, 14);
    
    // Convert raw accelerometer data (16-bit signed)
    int16_t rawAccelX = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t rawAccelY = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t rawAccelZ = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    // Convert raw gyroscope data
    int16_t rawGyroX = (int16_t)(buffer[7] << 8 | buffer[6]);
    int16_t rawGyroY = (int16_t)(buffer[9] << 8 | buffer[8]);
    int16_t rawGyroZ = (int16_t)(buffer[11] << 8 | buffer[10]);
    
    // Apply scale factors
    data.accelX = rawAccelX * accelScale;
    data.accelY = rawAccelY * accelScale;
    data.accelZ = rawAccelZ * accelScale;
    
    data.gyroX = rawGyroX * gyroScale;
    data.gyroY = rawGyroY * gyroScale;
    data.gyroZ = rawGyroZ * gyroScale;
    
    return true;
}

void LSM6DSOX_Driver::calibrateAccelerometer() {
    Serial.println("Calibrating accelerometer... Keep device still!");
    
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    const int SAMPLES = 100;
    
    for (int i = 0; i < SAMPLES; i++) {
        uint8_t buffer[6];
        readMultipleRegisters(OUTX_L_XL, buffer, 6);
        
        sumX += (int16_t)(buffer[1] << 8 | buffer[0]);
        sumY += (int16_t)(buffer[3] << 8 | buffer[2]);
        sumZ += (int16_t)(buffer[5] << 8 | buffer[4]);
        
        delay(10);
    }
    
    Serial.print("Calibration offsets - X: ");
    Serial.print(sumX / SAMPLES);
    Serial.print(" Y: ");
    Serial.print(sumY / SAMPLES);
    Serial.print(" Z: ");
    Serial.println(sumZ / SAMPLES);
}
