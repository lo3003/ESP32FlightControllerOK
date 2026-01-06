#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "imu.h"
#include "config.h"

Adafruit_MPU6050 mpu;

// Variables de calibration
float gyro_roll_cal = 0, gyro_pitch_cal = 0, gyro_yaw_cal = 0;
float acc_roll_cal = 0, acc_pitch_cal = 0; 

void imu_init() {
    if (!mpu.begin(GYRO_ADDRESS)) {
        Serial.println("ERREUR: MPU6050 introuvable !");
        while (1) { delay(10); } 
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Calibration (Ne pas bouger)...");
    
    long nb_samples = 2000;
    for (int i = 0; i < nb_samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        gyro_roll_cal += g.gyro.x;
        gyro_pitch_cal += g.gyro.y;
        gyro_yaw_cal += g.gyro.z;

        float acc_total = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
        
        // Calibration inversée (X <-> Y) pour correspondre au vol
        if(abs(a.acceleration.x) < acc_total) {
            acc_pitch_cal += asin(a.acceleration.x / acc_total) * 57.296;
        }
        if(abs(a.acceleration.y) < acc_total) {
            acc_roll_cal += asin(a.acceleration.y / acc_total) * -57.296;
        }

        if(i % 200 == 0) Serial.print(".");
        delay(2);
    }
    
    gyro_roll_cal /= nb_samples;
    gyro_pitch_cal /= nb_samples;
    gyro_yaw_cal /= nb_samples;
    
    acc_roll_cal /= nb_samples;
    acc_pitch_cal /= nb_samples;

    Serial.println(" OK");
}

void imu_read(DroneState *drone) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- 1. GYRO (ECHANGE X <-> Y et INVERSIONS) ---
    // Vos observations : Roll physique joue sur Pitch code -> On inverse X et Y
    // Roll : Utilise Gyro Y
    // Pitch : Utilise Gyro X
    
    float gyro_roll = (g.gyro.y - gyro_pitch_cal) * 57.296;  // On prend Y pour le Roll
    float gyro_pitch = (g.gyro.x - gyro_roll_cal) * 57.296;  // On prend X pour le Pitch
    float gyro_yaw = (g.gyro.z - gyro_yaw_cal) * 57.296;

    // Ajustement des signes selon vos tests (Inversion pour avoir le bon sens)
    gyro_roll *= 1;   // A tester : Si incliner à droite donne negatif, mettre -1
    gyro_pitch *= -1; // A tester : Si nez en haut donne negatif, mettre 1
    gyro_yaw *= -1;

    drone->gyro_roll_input = (drone->gyro_roll_input * 0.7) + (gyro_roll * 0.3);
    drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.7) + (gyro_pitch * 0.3);
    drone->gyro_yaw_input = (drone->gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);

    // 2. Integration
    drone->angle_pitch += gyro_pitch * 0.004;
    drone->angle_roll += gyro_roll * 0.004;

    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.004 * (3.142/180));
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.004 * (3.142/180));

    // --- 3. ACCEL (ECHANGE X <-> Y) ---
    float acc_total_vector = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
    
    float acc_weight = 0.01; 
    float gyro_weight = 0.99;

    // Calcul PITCH via ACCEL X (Swap)
    if(abs(a.acceleration.x) < acc_total_vector){
        // Signe inversé (-57.296) pour corriger "Nez bas = monte"
        float angle_pitch_acc = asin(a.acceleration.x / acc_total_vector) * 57.296;
        angle_pitch_acc -= acc_pitch_cal; 
        
        drone->angle_pitch = drone->angle_pitch * gyro_weight + angle_pitch_acc * acc_weight;
    }
    
    // Calcul ROLL via ACCEL Y (Swap)
    if(abs(a.acceleration.y) < acc_total_vector){
        // Signe positif (57.296) pour corriger "Droite = descend"
        float angle_roll_acc = asin(a.acceleration.y / acc_total_vector) * 57.296;
        angle_roll_acc -= acc_roll_cal;

        drone->angle_roll = drone->angle_roll * gyro_weight + angle_roll_acc * acc_weight;
    }
}