#include "pid.h"
#include "config.h"
#include <Arduino.h>

// --- VARIABLES MEMOIRES ---
// Note : pid_last_..._input stocke la dernière MESURE (Gyro) pour le D
float pid_i_mem_roll, pid_last_roll_input;
float pid_i_mem_pitch, pid_last_pitch_input;
float pid_i_mem_yaw, pid_last_yaw_input;

float pid_roll_d_filter_old;
float pid_pitch_d_filter_old;
float pid_yaw_d_filter_old;

// Timer de sécurité pour ne pas resetter le I sur une micro-coupure
static unsigned long pid_inflight_timer = 0;

// --- INITIALISATION DES PARAMETRES PID ---
// Appelé au démarrage pour charger les valeurs par défaut "No Stress"
void pid_init_params(DroneState *drone) {
    // ROLL/PITCH - Valeurs plus agressives pour un vrai vol
    drone->p_pitch_roll = 2.5;   // MODIFIÉ : était 1.5, maintenant plus réactif
    drone->i_pitch_roll = 0.01;  // MODIFIÉ : était 0, maintenant actif pour corriger les dérives
    drone->d_pitch_roll = 8.0;   // MODIFIÉ : était 5.0, plus d'amortissement

    // YAW
    drone->p_yaw = 2.0;          // MODIFIÉ : était 0.0
    drone->i_yaw = 0.005;        // MODIFIÉ : était 0.0
    drone->d_yaw = 0.0;

    // AUTO LEVEL
    drone->p_level = 5.0;        // MODIFIÉ : était 7.0, un peu moins agressif
    
    Serial.println("PID Params Initialized (Flight-Ready Mode)");
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0; pid_last_roll_input = 0; pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_input = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_input = 0; pid_yaw_d_filter_old = 0;
    pid_inflight_timer = 0; // Reset du timer
}

// --- CALCUL DES CONSIGNES (Setpoints) ---
void pid_compute_setpoints(DroneState *drone) {
    // --- ROLL ---
    float input_roll = 0;
    if(drone->channel_1 > 1503) input_roll = drone->channel_1 - 1503;
    else if(drone->channel_1 < 1497) input_roll = drone->channel_1 - 1497;
    
    input_roll -= drone->angle_roll * drone->p_level; 
    drone->pid_setpoint_roll = input_roll / 1.5;

    // --- PITCH ---
    float input_pitch = 0;
    if(drone->channel_2 > 1503) input_pitch = drone->channel_2 - 1503;
    else if(drone->channel_2 < 1497) input_pitch = drone->channel_2 - 1497;
    
    input_pitch -= drone->angle_pitch * drone->p_level; 
    drone->pid_setpoint_pitch = input_pitch / 1.5;

    // --- YAW ---
    float input_yaw = 0;
    if(drone->channel_3 > 1050) { 
        if(drone->channel_4 > 1503) input_yaw = (drone->channel_4 - 1500);
        else if(drone->channel_4 < 1497) input_yaw = (drone->channel_4 - 1500);
    }
    
    // ✅ CORRECTION CRITIQUE : RETIRER L'INVERSION
    drone->pid_setpoint_yaw = input_yaw;  // <-- PLUS de signe moins !
}

// --- BOUCLE PID PRINCIPALE ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;
    
    // 1. DETECTION VOL SECURISÉE
    if (drone->channel_3 > 1020) {
        pid_inflight_timer = millis();
    }
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2. TPA
    float tpa_factor = 1.0f;
    if (drone->channel_3 > 1500) {
        tpa_factor = map(drone->channel_3, 1500, 2000, 100, 80) / 100.0f;
    }

    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    
    // Calcul P et D d'abord
    float p_term_roll = (drone->p_pitch_roll * tpa_factor) * error;
    
    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    float d_term_roll = (drone->d_pitch_roll * tpa_factor) * d_err_filtered;

    // --- MODIFIÉ: Anti-windup conditionnel ---
    if (in_flight) {
        float output_before_i = p_term_roll + pid_i_mem_roll + d_term_roll;
        // On n'accumule le I que si la sortie n'est pas déjà saturée
        if (fabsf(output_before_i) < PID_MAX_ROLL) {
            pid_i_mem_roll += drone->i_pitch_roll * error;
        }
        // Saturation I
        if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
        else if (pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    } else {
        pid_i_mem_roll = 0;
    }

    // Sortie ROLL
    drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll;
    
    if (drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if (drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;

    float p_term_pitch = (drone->p_pitch_roll * tpa_factor) * error;

    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;
    float d_term_pitch = (drone->d_pitch_roll * tpa_factor) * d_err_filtered;

    // --- MODIFIÉ: Anti-windup conditionnel ---
    if (in_flight) {
        float output_before_i = p_term_pitch + pid_i_mem_pitch + d_term_pitch;
        if (fabsf(output_before_i) < PID_MAX_PITCH) {
            pid_i_mem_pitch += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
        else if (pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    } else {
        pid_i_mem_pitch = 0;
    }

    drone->pid_output_pitch = p_term_pitch + pid_i_mem_pitch + d_term_pitch;

    if (drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if (drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;

    float p_term_yaw = drone->p_yaw * error;

    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;
    float d_term_yaw = drone->d_yaw * d_err_filtered;

    // --- MODIFIÉ: Anti-windup conditionnel ---
    if (in_flight) {
        float output_before_i = p_term_yaw + pid_i_mem_yaw + d_term_yaw;
        if (fabsf(output_before_i) < PID_MAX_YAW) {
            pid_i_mem_yaw += drone->i_yaw * error;
        }
        if (pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
        else if (pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    } else {
        pid_i_mem_yaw = 0;
    }

    drone->pid_output_yaw = p_term_yaw + pid_i_mem_yaw + d_term_yaw;

    if (drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if (drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}