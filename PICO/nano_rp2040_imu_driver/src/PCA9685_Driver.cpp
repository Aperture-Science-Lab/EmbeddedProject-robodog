#include "PCA9685_Driver.h"

PCA9685_Driver::PCA9685_Driver(uint8_t address) {
    _address = address;
    _frequency = SERVO_FREQUENCY;
}

void PCA9685_Driver::writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t PCA9685_Driver::readByte(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool PCA9685_Driver::begin() {
    Wire.begin();
    delay(10);
    
    Wire.beginTransmission(_address);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.print("PCA9685 not found at 0x");
        Serial.println(_address, HEX);
        return false;
    }
    
    Serial.print("PCA9685 found at 0x");
    Serial.println(_address, HEX);
    
    reset();
    setPWMFrequency(SERVO_FREQUENCY);
    
    return true;
}

void PCA9685_Driver::reset() {
    writeByte(PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

void PCA9685_Driver::sleep() {
    uint8_t mode = readByte(PCA9685_MODE1);
    writeByte(PCA9685_MODE1, mode | MODE1_SLEEP);
}

void PCA9685_Driver::wakeup() {
    uint8_t mode = readByte(PCA9685_MODE1);
    writeByte(PCA9685_MODE1, mode & ~MODE1_SLEEP);
    delay(5);
}

void PCA9685_Driver::setPWMFrequency(uint16_t freq) {
    uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * freq) - 0.5f);
    
    uint8_t oldmode = readByte(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | MODE1_SLEEP;
    
    writeByte(PCA9685_MODE1, newmode);
    writeByte(PCA9685_PRESCALE, prescale);
    writeByte(PCA9685_MODE1, oldmode);
    delay(5);
    writeByte(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    
    _frequency = freq;
    
    Serial.print("PWM freq: ");
    Serial.print(freq);
    Serial.println(" Hz");
}

void PCA9685_Driver::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        Serial.println("Error: Channel 0-15");
        return;
    }
    
    uint8_t reg = PCA9685_LED0_ON_L + (4 * channel);
    
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

void PCA9685_Driver::setAllPWM(uint16_t on, uint16_t off) {
    Wire.beginTransmission(_address);
    Wire.write(0xFA);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

void PCA9685_Driver::setServoPulse(uint8_t channel, uint16_t pulse_us) {
    uint16_t steps = (uint16_t)((float)pulse_us / 20000.0f * 4096.0f);
    setPWM(channel, 0, steps);
}

void PCA9685_Driver::setServoAngle(uint8_t channel, float angle, 
                                    uint16_t minPulse, uint16_t maxPulse) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    uint16_t pulse = minPulse + (uint16_t)((angle / 180.0f) * (maxPulse - minPulse));
    setServoPulse(channel, pulse);
}

void PCA9685_Driver::disableChannel(uint8_t channel) {
    setPWM(channel, 0, 0);
}
