#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "config.h"
#include "motors.h" 

// Variables globales internes pour les offsets
double gyro_off_x = 0;
double gyro_off_y = 0;
double gyro_off_z = 0;

// Variables brutes
int16_t acc_raw[3];
int16_t gyro_raw[3];
int16_t temperature;

void imu_init() {
    // Sécurité moteurs
    motors_write_direct(1000, 1000, 1000, 1000);

    Serial.println(F("IMU: Init Raw I2C..."));
    
    // 1. Réveil & Config
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // 500 dps
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); // +/- 8g
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // 42Hz Filter (Standard)

    Serial.println(F("IMU: Calib Gyro (1000 samples)..."));
    digitalWrite(PIN_LED, HIGH);

    // 2. CALIBRATION (RAM)
    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < 1000 ; i++){
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);
        
        if(Wire.available() < 6) { delay(2); continue; }
        
        // Cast int16_t immédiat pour éviter le bug 32-bits
        gyro_sum_x += (int16_t)(Wire.read()<<8|Wire.read());
        gyro_sum_y += (int16_t)(Wire.read()<<8|Wire.read());
        gyro_sum_z += (int16_t)(Wire.read()<<8|Wire.read());
        
        delayMicroseconds(2000); 
    }

    gyro_off_x = gyro_sum_x / 1000.0;
    gyro_off_y = gyro_sum_y / 1000.0;
    gyro_off_z = gyro_sum_z / 1000.0;

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK"));
}

void imu_read(DroneState *drone) {
    // 1. Lecture Brute
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    
    uint8_t count = Wire.requestFrom(MPU_ADDR, 14);
    if(count < 14) return; 
    
    acc_raw[0] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc X
    acc_raw[1] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc Y
    acc_raw[2] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc Z
    temperature = (int16_t)(Wire.read()<<8|Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro X
    gyro_raw[1] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro Y
    gyro_raw[2] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro Z

    // 2. Offsets
    double gyro_x_cal = gyro_raw[0] - gyro_off_x;
    double gyro_y_cal = gyro_raw[1] - gyro_off_y;
    double gyro_z_cal = gyro_raw[2] - gyro_off_z;

    // =========================================================
    // 3. MAPPING AXES (CORRIGÉ POUR VOS CONVENTIONS)
    // =========================================================
    
    // ROLL (Standard) : Rotation autour de X. Incline l'axe Y.
    double gyro_roll  = gyro_x_cal; 
    long acc_roll_val = acc_raw[1]; // Accel Y mesure le Roll

    // PITCH (Inversé) : Rotation autour de Y. Incline l'axe X.
    // Votre convention : AnglePitch = -AngleY.
    // Donc Nez en Bas (Pitch Forward) = Rotation Y négative standard => Angle POSITIF ici.
    double gyro_pitch = -gyro_y_cal; 
    long acc_pitch_val = acc_raw[0]; // Accel X mesure le Pitch (X pointe vers le bas quand le nez baisse)

    // YAW : Standard Z
    double gyro_yaw   = gyro_z_cal; 
    long acc_yaw_val   = acc_raw[2]; // Z

    // 4. Filtrage Gyro (Rate)
    //float gyro_scale = 65.5; 
    //drone->gyro_roll_input  = (drone->gyro_roll_input * 0.7) + ((gyro_roll / gyro_scale) * 0.3);
    //drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.7) + ((gyro_pitch / gyro_scale) * 0.3);
    //drone->gyro_yaw_input   = (drone->gyro_yaw_input * 0.7) + ((gyro_yaw / gyro_scale) * 0.3);
    float gyro_scale = 65.5; 
    
    // --- CORRECTION : IL FAUT ENVOYER LA VALEUR AU PID ! ---
    // (J'ai décommenté et retiré le calcul de moyenne pour supprimer la latence)
    
    drone->gyro_roll_input  = (gyro_roll / gyro_scale);
    drone->gyro_pitch_input = (gyro_pitch / gyro_scale);
    drone->gyro_yaw_input   = (gyro_yaw / gyro_scale);

    // 5. Calcul Angle (Filtre Complémentaire)
    // Intégration
    drone->angle_pitch += gyro_pitch * GYRO_COEFF;
    drone->angle_roll  += gyro_roll * GYRO_COEFF;

    // Compensation Yaw
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll  += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    // Accéléromètre Vecteur Total
    drone->acc_total_vector = sqrt((acc_roll_val*acc_roll_val)+(acc_pitch_val*acc_pitch_val)+(acc_yaw_val*acc_yaw_val));

    // Calcul Angles Accel
    // Note : asin() renvoie des radians. On convertit en degrés.
    float angle_pitch_acc = 0, angle_roll_acc = 0;
    
    if(abs(acc_pitch_val) < drone->acc_total_vector){
        // Accel X positif quand le nez baisse -> Angle Pitch positif
        angle_pitch_acc = asin((float)acc_pitch_val / drone->acc_total_vector) * RAD_TO_DEG;
    }
    if(abs(acc_roll_val) < drone->acc_total_vector){
        // Accel Y positif quand l'aile droite baisse -> Angle Roll positif
        angle_roll_acc = asin((float)acc_roll_val / drone->acc_total_vector) * RAD_TO_DEG;
    }

    // Ajustement fin (Trim Accéléromètre à régler à plat si besoin)
    angle_roll_acc  += 3.0; 
    angle_pitch_acc += -6.0; 

    // Fusion
    if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
        drone->angle_pitch = drone->angle_pitch * 0.90 + angle_pitch_acc * 0.10;
        drone->angle_roll  = drone->angle_roll  * 0.90 + angle_roll_acc  * 0.10;
    } else {
        drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        drone->angle_roll  = drone->angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;
    }
}