#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

#define PCA9685_ADDRESS 0x40
#define PCA9685_MODE1      0x00
#define PCA9685_MODE2      0x01
#define PCA9685_PRESCALE   0xFE
#define PCA9685_LED0_ON_L  0x06

#define MODE1_RESTART 0x80
#define MODE1_SLEEP   0x10
#define MODE1_AI      0x20

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_FREQUENCY 50

class PCA9685_Driver {
private:
    uint8_t _address;
    uint16_t _frequency;
    
    void writeByte(uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t reg);

public:
    PCA9685_Driver(uint8_t address = PCA9685_ADDRESS);
    
    bool begin();
    void reset();
    void sleep();
    void wakeup();
    
    void setPWMFrequency(uint16_t freq);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setAllPWM(uint16_t on, uint16_t off);
    
    void setServoPulse(uint8_t channel, uint16_t pulse_us);
    void setServoAngle(uint8_t channel, float angle, 
                       uint16_t minPulse = SERVO_MIN_PULSE, 
                       uint16_t maxPulse = SERVO_MAX_PULSE);
    void disableChannel(uint8_t channel);
};

#endif
