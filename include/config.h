#ifndef CONFIG_H
#define CONFIG_H

// --- PINOUT (ESP32 30-PIN) ---
// Adapté à votre câblage physique :
#define PIN_MOTOR_1    27  // Avant Droit (CCW)
#define PIN_MOTOR_2    13  // Arrière Droit (CW)
#define PIN_MOTOR_3    25  // Arrière Gauche (CCW)
#define PIN_MOTOR_4    26  // Avant Gauche (CW)


#define ESC_FREQ       250 

// --- RADIO (S.BUS) ---
#define PIN_SBUS_RX    4   
#define D_FILTER_COEFF 0.3f  
#define PIN_LED        5

// --- I2C ---
#define GYRO_ADDRESS   0x68
#define I2C_SPEED      400000

// --- PARAMETRES DE VOL (PID) ---
#define PID_P_ROLL     1.5
#define PID_I_ROLL     0.015
#define PID_D_ROLL     5
#define PID_MAX_ROLL   400

#define PID_P_PITCH    1.5
#define PID_I_PITCH    0.015
#define PID_D_PITCH    5
#define PID_MAX_PITCH  400

#define PID_P_YAW      0
#define PID_I_YAW      0
#define PID_D_YAW      0
#define PID_MAX_YAW    400

// --- REGLAGES MOTEURS ---
#define MAX_THROTTLE_FLIGHT 1800 
#define MIN_THROTTLE_IDLE   1050
#define MOTOR_OFF           1000

#define LOOP_TIME_US   4000 

#endif