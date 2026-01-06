#include "pid.h"
#include "config.h"

float pid_i_mem_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_last_yaw_d_error;

float pid_roll_d_filter_old;
float pid_pitch_d_filter_old;
float pid_yaw_d_filter_old;

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
    input -= drone->angle_roll * 15; // Self-Level
    drone->pid_setpoint_roll = input / 3.0;

    // Calcul Pitch
    input = 0;
    if(drone->channel_2 > 1508) input = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input = drone->channel_2 - 1492;
    input -= drone->angle_pitch * 15;
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
    pid_i_mem_roll += PID_I_ROLL * error;
    if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    
    // Calcul D avec FILTRE VFINAL
    d_err_raw = error - pid_last_roll_d_error;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_d_error = error;
    
    drone->pid_output_roll = PID_P_ROLL * error + pid_i_mem_roll + PID_D_ROLL * d_err_filtered;
    
    if(drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if(drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // --- PITCH ---
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    pid_i_mem_pitch += PID_I_PITCH * error;
    if(pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
    else if(pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    
    // Calcul D avec FILTRE
    d_err_raw = error - pid_last_pitch_d_error;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_d_error = error;

    drone->pid_output_pitch = PID_P_PITCH * error + pid_i_mem_pitch + PID_D_PITCH * d_err_filtered;

    if(drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if(drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // --- YAW ---
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    pid_i_mem_yaw += PID_I_YAW * error;
    if(pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
    else if(pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    
    // Calcul D avec FILTRE
    d_err_raw = error - pid_last_yaw_d_error;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_d_error = error;

    drone->pid_output_yaw = PID_P_YAW * error + pid_i_mem_yaw + PID_D_YAW * d_err_filtered;

    if(drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if(drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}