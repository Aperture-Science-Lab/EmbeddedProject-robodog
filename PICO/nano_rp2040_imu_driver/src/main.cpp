#include <Arduino.h>
#include "LSM6DSOX_Driver.h"

LSM6DSOX_Driver imu;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("Initializing LSM6DSOX IMU...");
    
    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    
    Serial.println("IMU initialized successfully!");
    imu.calibrateAccelerometer();
}

void loop() {
    LSM6DSOX_Driver::SensorData data;
    
    if (imu.readSensorData(data)) {
        Serial.print("Accel: ");
        Serial.print(data.accelX); Serial.print(", ");
        Serial.print(data.accelY); Serial.print(", ");
        Serial.print(data.accelZ); Serial.println(" mg");
        
        Serial.print("Gyro: ");
        Serial.print(data.gyroX); Serial.print(", ");
        Serial.print(data.gyroY); Serial.print(", ");
        Serial.print(data.gyroZ); Serial.println(" dps");
    }
    
    delay(100);
}
