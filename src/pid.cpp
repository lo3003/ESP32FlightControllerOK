#include "pid.h"
#include "config.h"

float pid_i_mem_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_last_yaw_d_error;

float pid_roll_d_filter_old;
float pid_pitch_d_filter_old;
float pid_yaw_d_filter_old;

// Charge les valeurs par défaut du config.h dans les variables modifiables
void pid_init_params(DroneState *drone) {
    drone->p_pitch_roll = PID_P_ROLL;
    drone->i_pitch_roll = PID_I_ROLL;
    drone->d_pitch_roll = PID_D_ROLL;

    drone->p_yaw = PID_P_YAW;
    drone->i_yaw = PID_I_YAW;
    drone->d_yaw = PID_D_YAW;

    drone->p_level = 15.0; // Valeur par défaut (hardcodée avant)
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0; pid_last_roll_d_error = 0; pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_d_error = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_d_error = 0; pid_yaw_d_filter_old = 0;
}

void pid_compute_setpoints(DroneState *drone) {
    // Calcul Roll
    float input = 0;
    if(drone->channel_1 > 1508) input = drone->channel_1 - 1508;
    else if(drone->channel_1 < 1492) input = drone->channel_1 - 1492;
    
    input -= drone->angle_roll * drone->p_level; // UTILISATION VARIABLE LEVEL
    drone->pid_setpoint_roll = input / 3.0;

    // Calcul Pitch
    input = 0;
    if(drone->channel_2 > 1508) input = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input = drone->channel_2 - 1492;
    
    input -= drone->angle_pitch * drone->p_level; // UTILISATION VARIABLE LEVEL
    drone->pid_setpoint_pitch = input / 3.0;

    // Calcul Yaw
    input = 0;
    if(drone->channel_3 > 1050) {
        if(drone->channel_4 > 1508) input = (drone->channel_4 - 1508) / 3.0;
        else if(drone->channel_4 < 1492) input = (drone->channel_4 - 1492) / 3.0;
    }
    drone->pid_setpoint_yaw = input;
}

void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // --- ROLL ---
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    pid_i_mem_roll += drone->i_pitch_roll * error; // VARIABLE I
    
    if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    
    d_err_raw = error - pid_last_roll_d_error;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_d_error = error;
    
    // FORMULE PID AVEC VARIABLES
    drone->pid_output_roll = drone->p_pitch_roll * error + pid_i_mem_roll + drone->d_pitch_roll * d_err_filtered;
    
    if(drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if(drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // --- PITCH ---
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    pid_i_mem_pitch += drone->i_pitch_roll * error; // VARIABLE I (Partagée)
    
    if(pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
    else if(pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    
    d_err_raw = error - pid_last_pitch_d_error;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_d_error = error;

    // FORMULE PID AVEC VARIABLES (P, D Partagés)
    drone->pid_output_pitch = drone->p_pitch_roll * error + pid_i_mem_pitch + drone->d_pitch_roll * d_err_filtered;

    if(drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if(drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // --- YAW ---
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    pid_i_mem_yaw += drone->i_yaw * error; // VARIABLE I YAW
    
    if(pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
    else if(pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    
    d_err_raw = error - pid_last_yaw_d_error;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_d_error = error;

    // FORMULE PID AVEC VARIABLES YAW
    drone->pid_output_yaw = drone->p_yaw * error + pid_i_mem_yaw + drone->d_yaw * d_err_filtered;

    if(drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if(drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}