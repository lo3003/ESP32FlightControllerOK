#ifndef IMU_H
#define IMU_H
#include "types.h"
#include <Arduino.h> // Nécessaire pour les types byte/int16_t

// Adresse I2C standard du MPU6050
#define MPU_ADDR 0x68

// Coefficients pour le passage Raw -> Degrés
// Pour une boucle à 250Hz (4000us) et Gyro 500dps
#define GYRO_COEFF 0.0000611

// Conversion Radian -> Degré
#define RAD_TO_DEG 57.296

void imu_init();
void imu_read(DroneState *drone);

#endif