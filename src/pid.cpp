#include "pid.h"
#include "config.h"
#include <Arduino.h>

// Variables mémoires statiques
// Note : pid_last_..._d_error stockera désormais la dernière MESURE (Input) et non l'erreur
float pid_i_mem_roll, pid_last_roll_input;
float pid_i_mem_pitch, pid_last_pitch_input;
float pid_i_mem_yaw, pid_last_yaw_input;

float pid_roll_d_filter_old;
float pid_pitch_d_filter_old;
float pid_yaw_d_filter_old;

// Initialisation des paramètres avec les valeurs "Safe Start" recommandées
void pid_init_params(DroneState *drone) {
    // --- REGLAGES "SAFE START" POUR ESP32 ---
    // Ces valeurs remplacent temporairement celles de config.h
    
    // ROLL
    drone->p_pitch_roll = 1.0;   // Réactivité douce (au lieu de 1.5)
    drone->i_pitch_roll = 0.01;  // Maintien minimal
    drone->d_pitch_roll = 5.0;   // Amortissement moyen (au lieu de 18 ou 3)

    // PITCH (Mêmes valeurs que Roll)
    // Note : P/I/D sont partagés dans la struct pour Roll/Pitch, mais on pourrait les séparer
    // Ici on utilise les vars p_pitch_roll définies dans types.h

    // YAW
    drone->p_yaw = 4.0;          // Valeur standard Arduino
    drone->i_yaw = 0.02;
    drone->d_yaw = 0.0;

    // AUTO LEVEL (Angle)
    drone->p_level = 2.0;        // Retour doux (au lieu de 15.0 qui était violent)
    
    Serial.println("PID Params Initialized (Safe Defaults)");
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0; pid_last_roll_input = 0; pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_input = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_input = 0; pid_yaw_d_filter_old = 0;
}

void pid_compute_setpoints(DroneState *drone) {
    // --- ROLL ---
    float input = 0;
    // Deadband de 16us (1492-1508)
    if(drone->channel_1 > 1508) input = drone->channel_1 - 1508;
    else if(drone->channel_1 < 1492) input = drone->channel_1 - 1492;
    
    // Calcul de l'angle cible
    // On garde le diviseur 3.0 de l'ESP32. Si c'est trop sensible, passez à 4.0 ou 5.0
    input -= drone->angle_roll * drone->p_level; 
    drone->pid_setpoint_roll = input / 3.0;

    // --- PITCH ---
    input = 0;
    if(drone->channel_2 > 1508) input = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input = drone->channel_2 - 1492;
    
    input -= drone->angle_pitch * drone->p_level; 
    drone->pid_setpoint_pitch = input / 3.0;

    // --- YAW ---
    input = 0;
    if(drone->channel_3 > 1050) { // Uniquement si on a des gaz
        if(drone->channel_4 > 1508) input = (drone->channel_4 - 1508) / 3.0;
        else if(drone->channel_4 < 1492) input = (drone->channel_4 - 1492) / 3.0;
    }
    drone->pid_setpoint_yaw = input;
}

void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    
    // Terme I
    pid_i_mem_roll += drone->i_pitch_roll * error;
    if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    
    // --- AMELIORATION : D sur MESURE (Measurement) ---
    // Au lieu de (error - last_error), on fait (last_input - input)
    // Cela évite le "Derivative Kick" quand on bouge le stick.
    // Mathématiquement : d(Error)/dt = d(Setpoint - Input)/dt. Si Setpoint stable => -d(Input)/dt
    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    
    // Filtre du D (Low Pass Filter)
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    
    // Mise à jour mémoire pour le prochain tour
    pid_last_roll_input = drone->gyro_roll_input;
    
    // Calcul Sortie PID
    drone->pid_output_roll = drone->p_pitch_roll * error + pid_i_mem_roll + drone->d_pitch_roll * d_err_filtered;
    
    if(drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if(drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    
    pid_i_mem_pitch += drone->i_pitch_roll * error; 
    if(pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
    else if(pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    
    // D sur Mesure
    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;

    drone->pid_output_pitch = drone->p_pitch_roll * error + pid_i_mem_pitch + drone->d_pitch_roll * d_err_filtered;

    if(drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if(drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    
    pid_i_mem_yaw += drone->i_yaw * error; 
    if(pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
    else if(pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    
    // D sur Mesure (aussi valide pour le Yaw)
    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;

    drone->pid_output_yaw = drone->p_yaw * error + pid_i_mem_yaw + drone->d_yaw * d_err_filtered;

    if(drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if(drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}