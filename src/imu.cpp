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
    // Sécurité moteurs au démarrage
    motors_write_direct(1000, 1000, 1000, 1000);

    Serial.println(F("IMU: Init Raw I2C..."));
    
    // 1. Réveil & Config du MPU6050
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // 500 dps
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); // +/- 8g
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // 42Hz Filter (Standard)

    Serial.println(F("IMU: Calib Gyro (1000 samples)..."));
    digitalWrite(PIN_LED, HIGH);

    // 2. CALIBRATION GYRO (RAM)
    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < 1000 ; i++){
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);
        
        // Attente active sécurisée
        if(Wire.available() < 6) { delay(2); continue; }
        
        // Lecture et somme
        gyro_sum_x += (int16_t)(Wire.read()<<8|Wire.read());
        gyro_sum_y += (int16_t)(Wire.read()<<8|Wire.read());
        gyro_sum_z += (int16_t)(Wire.read()<<8|Wire.read());
        
        delayMicroseconds(2000); 
    }

    // Calcul des moyennes (Offsets)
    gyro_off_x = gyro_sum_x / 1000.0;
    gyro_off_y = gyro_sum_y / 1000.0;
    gyro_off_z = gyro_sum_z / 1000.0;

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK"));
}

void imu_read(DroneState *drone) {
    // 1. Demande lecture Brute (Acc + Temp + Gyro = 14 octets)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    
    uint8_t count = Wire.requestFrom(MPU_ADDR, 14);

    // --- SECURITE ANTI-CRASH (I2C FREEZE) ---
    // Si on a reçu moins de 14 octets, c'est que le bus I2C a planté.
    // On quitte immédiatement pour ne pas lire de données corrompues.
    if(count < 14) { 
        // On signale l'erreur dans la variable de diagnostic pour le voir dans l'interface Web
        // (Valeur arbitraire > 10000 signifiant "ERREUR HARDWARE")
        drone->max_time_imu = 888888; 
        return; 
    }
    
    // 2. Lecture des registres (Si tout va bien)
    acc_raw[0] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc X
    acc_raw[1] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc Y
    acc_raw[2] = (int16_t)(Wire.read()<<8|Wire.read()); // Acc Z
    temperature = (int16_t)(Wire.read()<<8|Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro X
    gyro_raw[1] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro Y
    gyro_raw[2] = (int16_t)(Wire.read()<<8|Wire.read()); // Gyro Z

    // 3. Application des Offsets (Calibration)
    double gyro_x_cal = gyro_raw[0] - gyro_off_x;
    double gyro_y_cal = gyro_raw[1] - gyro_off_y;
    double gyro_z_cal = gyro_raw[2] - gyro_off_z;

    // =========================================================
    // 4. MAPPING AXES (Adapté à la position physique du capteur)
    // =========================================================
    
    // ROLL : Rotation autour de X
    double gyro_roll  = gyro_x_cal; 
    long acc_roll_val = acc_raw[1]; // Accel Y

    // PITCH : Rotation autour de Y (Inversé selon convention)
    double gyro_pitch = -gyro_y_cal; 
    long acc_pitch_val = acc_raw[0]; // Accel X

    // YAW : Rotation autour de Z
    double gyro_yaw   = gyro_z_cal; 
    long acc_yaw_val  = acc_raw[2]; // Accel Z

    // 5. Conversion Échelle (LSB vers Degrés/seconde)
    // Pour 500dps, l'échelle est 65.5 LSB/°/s
    float gyro_scale = 65.5; 
    
    drone->gyro_roll_input  = (gyro_roll / gyro_scale);
    drone->gyro_pitch_input = (gyro_pitch / gyro_scale);
    drone->gyro_yaw_input   = (gyro_yaw / gyro_scale);

    // 6. Calcul Angle (Filtre Complémentaire)
    // Intégration Gyro
    drone->angle_pitch += gyro_pitch * GYRO_COEFF;
    drone->angle_roll  += gyro_roll * GYRO_COEFF;

    // Compensation Yaw (Transfet d'angle quand on tourne sur le lacet)
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll  += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    // Calcul Vecteur Accélération Totale
    drone->acc_total_vector = sqrt((acc_roll_val*acc_roll_val)+(acc_pitch_val*acc_pitch_val)+(acc_yaw_val*acc_yaw_val));

    // Calcul Angles Accéléromètre (Correction de dérive)
    float angle_pitch_acc = 0, angle_roll_acc = 0;
    
    if(abs(acc_pitch_val) < drone->acc_total_vector){
        angle_pitch_acc = asin((float)acc_pitch_val / drone->acc_total_vector) * RAD_TO_DEG;
    }
    if(abs(acc_roll_val) < drone->acc_total_vector){
        angle_roll_acc = asin((float)acc_roll_val / drone->acc_total_vector) * RAD_TO_DEG;
    }

    // Ajustement fin (Trim Accéléromètre) - A ajuster selon votre niveau à bulle
    angle_roll_acc  += 3.0; // Correction mécanique
    angle_pitch_acc += -6.0; 

    // 7. Fusion Gyro + Accel
    if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
        // Au sol : On fait confiance à l'accéléromètre pour s'aligner vite
        drone->angle_pitch = drone->angle_pitch * 0.90 + angle_pitch_acc * 0.10;
        drone->angle_roll  = drone->angle_roll  * 0.90 + angle_roll_acc  * 0.10;
    } else {
        // En vol : On fait confiance au Gyro (Filtre Complémentaire standard)
        drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        drone->angle_roll  = drone->angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;
    }
}