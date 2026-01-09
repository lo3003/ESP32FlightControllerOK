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
    // ROLL
    drone->p_pitch_roll = 1.5;   // Doux
    drone->i_pitch_roll = 0;  // Faible pour éviter le windup au début
    drone->d_pitch_roll = 5.0;   // Amortissement

    // PITCH (Copie du Roll)
    // Les variables P/I/D sont partagées pour Roll/Pitch dans votre struct

    // YAW
    drone->p_yaw = 0.0;          // Standard
    drone->i_yaw = 0.0;
    drone->d_yaw = 0.0;

    // AUTO LEVEL (Stabilisation Angle)
    drone->p_level = 7.0;        // Retour à plat progressif (3.0 max conseillé au début)
    
    Serial.println("PID Params Initialized (Smart & Safe Mode)");
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
    // Deadband (Zone morte centrale) pour éviter la dérive stick lâché
    if(drone->channel_1 > 1508) input_roll = drone->channel_1 - 1508;
    else if(drone->channel_1 < 1492) input_roll = drone->channel_1 - 1492;
    
    // Mode Angle : La consigne de vitesse dépend de l'angle actuel
    // Formule : (Stick - Angle * P_Level) = Vitesse de rotation demandée
    input_roll -= drone->angle_roll * drone->p_level; 
    drone->pid_setpoint_roll = input_roll / 3.0; // Diviseur sensibilité

    // --- PITCH ---
    float input_pitch = 0;
    if(drone->channel_2 > 1508) input_pitch = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input_pitch = drone->channel_2 - 1492;
    
    input_pitch -= drone->angle_pitch * drone->p_level; 
    drone->pid_setpoint_pitch = input_pitch / 3.0;

    // --- YAW ---
    float input_yaw = 0;
    // On n'autorise le Yaw que si on a un peu de gaz
    if(drone->channel_3 > 1050) { 
        if(drone->channel_4 > 1508) input_yaw = (drone->channel_4 - 1508) / 3.0;
        else if(drone->channel_4 < 1492) input_yaw = (drone->channel_4 - 1492) / 3.0;
    }
    drone->pid_setpoint_yaw = input_yaw;
}

// --- BOUCLE PID PRINCIPALE ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;
    
    // 1. DETECTION VOL SECURISÉE (Smart Integrator with Hysteresis)
    // Si Throttle > 1020, on met à jour le timer "Dernière fois vu en vol"
    if (drone->channel_3 > 1020) {
        pid_inflight_timer = millis();
    }

    // On considère qu'on vole tant que le dernier signal valide date de moins de 500ms
    // Cela permet de survivre à une micro-coupure radio sans reset violent du PID
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2. TPA (Throttle PID Attenuation)
    // Réduit les gains P et D quand les gaz augmentent pour éviter les oscillations
    float tpa_factor = 1.0;
    if (drone->channel_3 > 1500) {
        // Mappe 1500-2000 vers 1.0 -> 0.80 (20% de réduction à fond)
        tpa_factor = map(drone->channel_3, 1500, 2000, 100, 80) / 100.0;
    }

    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    
    // Terme I (SMART INTEGRATOR)
    if (in_flight) {
        pid_i_mem_roll += drone->i_pitch_roll * error;
        // Saturation I
        if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
        else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    } else {
        pid_i_mem_roll = 0; // Reset seulement si au sol depuis > 500ms
    }
    
    // Terme D (Measurement)
    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    // Filtrage D
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    
    // Sortie ROLL (Avec TPA sur P et D)
    drone->pid_output_roll = (drone->p_pitch_roll * tpa_factor) * error 
                           + pid_i_mem_roll 
                           + (drone->d_pitch_roll * tpa_factor) * d_err_filtered;
    
    // Saturation Globale
    if(drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if(drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    
    if (in_flight) {
        pid_i_mem_pitch += drone->i_pitch_roll * error; 
        if(pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
        else if(pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    } else {
        pid_i_mem_pitch = 0;
    }
    
    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;

    // Sortie PITCH (Avec TPA)
    drone->pid_output_pitch = (drone->p_pitch_roll * tpa_factor) * error 
                            + pid_i_mem_pitch 
                            + (drone->d_pitch_roll * tpa_factor) * d_err_filtered;

    if(drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if(drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    // Pas de TPA sur le Yaw généralement (ou moins nécessaire)
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    
    if (in_flight) {
        pid_i_mem_yaw += drone->i_yaw * error; 
        if(pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
        else if(pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    } else {
        pid_i_mem_yaw = 0;
    }
    
    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;

    drone->pid_output_yaw = drone->p_yaw * error + pid_i_mem_yaw + drone->d_yaw * d_err_filtered;

    if(drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if(drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}