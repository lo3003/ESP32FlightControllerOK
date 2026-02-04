#ifndef CONFIG_H
#define CONFIG_H

// --- PINOUT (ESP32 30-PIN) ---
// Adapté à votre câblage physique :
#define PIN_MOTOR_1    27
#define PIN_MOTOR_2    13
#define PIN_MOTOR_3    25
#define PIN_MOTOR_4    26

// --- BATTERIE ---
#define PIN_BATTERY    34
#define BAT_SCALE      5.88f  // Calibré: 12.19V mesurés pour 2.09V lus

#define ESC_FREQ       250 

// --- RADIO (S.BUS) ---
#define PIN_SBUS_RX    4   
#define D_FILTER_COEFF 0.45f 
#define PIN_LED        5

// --- I2C ---
#define GYRO_ADDRESS   0x68
#define I2C_SPEED      400000

// Mettre à 1 si ton yaw est juste dans le mauvais sens (commande pilote inversée)
#define IMU_INVERT_YAW 1
#define RC_INVERT_YAW  0

// --- PARAMETRES DE VOL (PID) ---
#define PID_P_ROLL     1.35
#define PID_I_ROLL     0.021
#define PID_D_ROLL     2
#define PID_MAX_ROLL   400

#define PID_P_PITCH    1.35
#define PID_I_PITCH    0.021
#define PID_D_PITCH    2
#define PID_MAX_PITCH  400

#define PID_P_YAW      2
#define PID_I_YAW      0.01
#define PID_D_YAW      1.5
#define PID_MAX_YAW    400

// --- REGLAGES MOTEURS ---
#define MAX_THROTTLE_FLIGHT 2000
#define MIN_THROTTLE_IDLE   1050
#define MOTOR_OFF           1000

#define LOOP_TIME_US   4000 

// --- IMU Calibration tuning ---
#define IMU_CALIB_SETTLE_MS     200     // temps après config (stabilisation capteur)
#define IMU_CALIB_DISCARD       50      // lectures jetées au début
#define IMU_CALIB_SAMPLES       1500    // échantillons utiles (plus = plus stable)
#define IMU_CALIB_DELAY_US      2000    // spacing entre échantillons
#define IMU_CALIB_STD_MAX_RAW   8.0f    // seuil "ça bouge" (en unités brutes gyro)

#define PIN_FLOW_RX    16  // Connecté au TX du Matek
#define PIN_FLOW_TX    17  // Connecté au RX du Matek
#define FLOW_BAUD      115200


#endif