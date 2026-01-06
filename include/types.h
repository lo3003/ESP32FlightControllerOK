#ifndef TYPES_H
#define TYPES_H

typedef enum {
    MODE_SAFE,           // Moteurs coupés, LED Fixe
    MODE_CALIBRATION,    // Mode spécial Calibration ESC
    MODE_SETUP,          // Mode spécial Setup Wizard
    MODE_PRE_ARM,        // Attente armement
    MODE_ARMED,          // Moteurs au ralenti
    MODE_FLYING          // En vol PID actifs
} FlightMode;

typedef struct {
    FlightMode current_mode;
    
    // Radio (valeurs en us : 1000-2000)
    int channel_1; 
    int channel_2; 
    int channel_3; 
    int channel_4; 

    // IMU
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float angle_roll;
    float angle_pitch;
    float acc_total_vector;

    // PID
    float pid_setpoint_roll;
    float pid_setpoint_pitch;
    float pid_setpoint_yaw;
    
    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

} DroneState;

#endif