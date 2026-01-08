#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h> 
#include "imu.h"
#include "config.h"

MPU6050 mpu(Wire);

void imu_init() {
    Serial.println(F("IMU: Connexion..."));
    byte status = mpu.begin();
    Serial.print(F("IMU Status: ")); Serial.println(status);
    
    if(status != 0){
        Serial.println(F("ERREUR IMU ! Stop."));
        while(1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
    }

    Serial.println(F("IMU: Calibration (NE PAS BOUGER LE DRONE)..."));
    digitalWrite(PIN_LED, HIGH); 
    delay(1000);
    mpu.calcOffsets(); 
    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration Terminee !"));
}

void imu_read(DroneState *drone) {
    // 1. On met à jour le calcul interne de la librairie
    mpu.update();
    
    drone->angle_roll  = mpu.getAngleX(); 
    drone->angle_pitch = -mpu.getAngleY(); 
    
    // Pour le PID, on a besoin de la vitesse angulaire (Gyro pur)
    drone->gyro_roll_input  = mpu.getGyroX();
    drone->gyro_pitch_input = -mpu.getGyroY();
    drone->gyro_yaw_input   = mpu.getGyroZ();
}