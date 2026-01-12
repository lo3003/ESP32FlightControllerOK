#ifndef IMU_H
#define IMU_H
#include "types.h"
#include <Arduino.h>

// Adresse I2C standard du MPU6050
#define MPU_ADDR 0x68

// Pour une boucle à 250Hz (4000us) et Gyro 500dps
#define GYRO_COEFF 0.0000611
#define RAD_TO_DEG 57.296

void imu_init();
void imu_read(DroneState *drone);

// --- AJOUT: IMU en tâche FreeRTOS (zéro impact loop) ---
void imu_start_task();
void imu_update(DroneState *drone);
void imu_request_reset();   // <-- AJOUT

#endif