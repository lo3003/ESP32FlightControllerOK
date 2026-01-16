#include "pid.h"
#include "config.h"
#include <Arduino.h>

// --- MEMOIRES PID ---
// Note : pid_last_..._input stocke la dernière MESURE (Gyro) pour le D
static float pid_i_mem_roll,  pid_last_roll_input;
static float pid_i_mem_pitch, pid_last_pitch_input;
static float pid_i_mem_yaw,   pid_last_yaw_input;

static float pid_roll_d_filter_old;
static float pid_pitch_d_filter_old;
static float pid_yaw_d_filter_old;

// FeedForward : consigne "pilote" (rate) mémorisée depuis pid_compute_setpoints()
static float ff_sp_roll = 0.0f;
static float ff_sp_pitch = 0.0f;
static float ff_sp_yaw = 0.0f;

// Timer de sécurité pour ne pas resetter le I sur une micro-coupure radio
static unsigned long pid_inflight_timer = 0;

// --- INITIALISATION DES PARAMETRES PID ---
// Appelé au démarrage pour charger des valeurs par défaut
void pid_init_params(DroneState *drone) {
    // ROLL/PITCH
    drone->p_pitch_roll = 2.5f;
    drone->i_pitch_roll = 0.01f;
    drone->d_pitch_roll = 8.0f;

    // YAW (rate)
    drone->p_yaw = 2.0f;
    drone->i_yaw = 0.005f;
    drone->d_yaw = 0.0f;

    // FEEDFORWARD (à tuner via Web)
    // Ordre de grandeur: 0.05 à 0.25
    drone->ff_pitch_roll = 0.16f;
    drone->ff_yaw        = 0.10f;

    // AUTO LEVEL
    drone->p_level = 5.0f;

    Serial.println("PID Params Initialized (Flight-Ready Mode)");
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0;  pid_last_roll_input = 0;  pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_input = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0;   pid_last_yaw_input = 0;   pid_yaw_d_filter_old = 0;
    pid_inflight_timer = 0;

    ff_sp_roll = ff_sp_pitch = ff_sp_yaw = 0.0f;
}

// --- CALCUL DES CONSIGNES (Setpoints) ---
// roll/pitch : "self-level" -> consigne rate = stick - angle*p_level
// yaw        : rate pure (pas d'angle yaw)
void pid_compute_setpoints(DroneState *drone) {
    // --- ROLL (stick) ---
    float stick_roll = 0.0f;
    if (drone->channel_1 > 1520)      stick_roll = (float)(drone->channel_1 - 1520);
    else if (drone->channel_1 < 1480) stick_roll = (float)(drone->channel_1 - 1480);

    // FeedForward = uniquement la commande pilote (rate)
    ff_sp_roll = stick_roll / 3.0f;

    // Self-level (outer loop)
    float input_roll = stick_roll - (drone->angle_roll * drone->p_level);
    drone->pid_setpoint_roll = input_roll / 3.0f;

    // --- PITCH (stick) ---
    float stick_pitch = 0.0f;
    if (drone->channel_2 > 1520)      stick_pitch = (float)(drone->channel_2 - 1520);
    else if (drone->channel_2 < 1480) stick_pitch = (float)(drone->channel_2 - 1480);

    ff_sp_pitch = stick_pitch / 3.0f;

    float input_pitch = stick_pitch - (drone->angle_pitch * drone->p_level);
    drone->pid_setpoint_pitch = input_pitch / 3.0f;

    // --- YAW (stick, rate only) ---
    float stick_yaw = 0.0f;

    // Autorise yaw seulement si on est au-dessus d'un minimum de gaz
    if (drone->channel_3 > 1050) {
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);
    }

#if RC_INVERT_YAW
    stick_yaw = -stick_yaw;
#endif

    ff_sp_yaw = stick_yaw;            // yaw déjà en "rate"
    drone->pid_setpoint_yaw = stick_yaw;
}

// --- BOUCLE PID PRINCIPALE ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // 1) DETECTION "IN FLIGHT"
    if (drone->channel_3 > 1020) {
        pid_inflight_timer = millis();
    }
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2) TPA (atténuation P/D/FF à haut gaz)
    float tpa_factor = 1.0f;
    if (drone->channel_3 > 1500) {
        //tpa_factor = map(drone->channel_3, 1500, 2000, 100, 80) / 100.0f;
        tpa_factor = 1.0f;
    }

    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;

    float p_term_roll = (drone->p_pitch_roll * tpa_factor) * error;

    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    float d_term_roll = (drone->d_pitch_roll * tpa_factor) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_roll + pid_i_mem_roll + d_term_roll;
        if (fabsf(output_before_i) < PID_MAX_ROLL) {
            pid_i_mem_roll += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
        else if (pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    } else {
        pid_i_mem_roll = 0;
    }

    float ff_term_roll = (drone->ff_pitch_roll * tpa_factor) * ff_sp_roll;

    drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll + ff_term_roll;
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

    float ff_term_pitch = (drone->ff_pitch_roll * tpa_factor) * ff_sp_pitch;

    drone->pid_output_pitch = p_term_pitch + pid_i_mem_pitch + d_term_pitch + ff_term_pitch;
    if (drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if (drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    // yaw = rate PID pur (aucun angle yaw)
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;

    float p_term_yaw = (drone->p_yaw * tpa_factor) * error;

    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;
    float d_term_yaw = (drone->d_yaw * tpa_factor) * d_err_filtered;

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

    float ff_term_yaw = (drone->ff_yaw * tpa_factor) * ff_sp_yaw;

    drone->pid_output_yaw = p_term_yaw + pid_i_mem_yaw + d_term_yaw + ff_term_yaw;
    if (drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if (drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}
