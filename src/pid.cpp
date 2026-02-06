#include "pid.h"
#include "config.h"
#include <Arduino.h>

// --- TENSION DE REFERENCE POUR LA COMPENSATION (A ajuster selon ta batterie) ---
// 12.0f pour 3S (11.1V nominal, 12.6V max)
// 16.0f pour 4S (14.8V nominal, 16.8V max)
#define PID_REF_VOLTAGE 12.6f 

// --- FLAG POUR DESACTIVER LE HEADING HOLD ---
// Mettre à 1 pour activer, 0 pour désactiver (mode simple)
#define HEADING_HOLD_ENABLED 1

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

// --- HEADING HOLD (Maintien de Cap) ---
static float yaw_target_angle = 0.0f;   // Cap cible mémorisé
static bool yaw_heading_lock = false;   // Flag: lock de cap actif
static float last_valid_yaw = 0.0f;     // Dernier yaw valide mémorisé

// --- ALTITUDE HOLD (PID mémoire) ---
static float pid_i_mem_alt = 0.0f;
static float pid_last_alt_error = 0.0f;

// --- POSITION HOLD (Optical Flow - Variables supprimées, maintenant dans DroneState) ---
// Les variables position_x/y, anchor_x/y, vel_integral, vel_prev_err sont dans DroneState

// Paramètres Heading Hold
static const float YAW_DEADBAND = 20.0f;        // Deadband stick yaw (±20 autour de 1500)
static const float YAW_RATE_LIMIT = 200.0f;     // Limite max de la consigne rate générée (deg/s)
static const float YAW_JUMP_THRESHOLD = 30.0f;  // Seuil de détection de saut aberrant (deg)

// Timer de sécurité pour ne pas resetter le I sur une micro-coupure radio
static unsigned long pid_inflight_timer = 0;

// --- INITIALISATION DES PARAMETRES PID ---
// Appelé au démarrage pour charger des valeurs par défaut
void pid_init_params(DroneState *drone) {
    // ROLL/PITCH
    drone->p_pitch_roll = PID_P_ROLL;
    drone->i_pitch_roll = PID_I_ROLL;
    drone->d_pitch_roll = PID_D_ROLL;

    // YAW (rate)
    drone->p_yaw = PID_P_YAW;
    drone->i_yaw = PID_I_YAW;
    drone->d_yaw = PID_D_YAW;

    // FEEDFORWARD
    drone->ff_pitch_roll = 0;
    drone->ff_yaw        = 0;

    // AUTO LEVEL
    drone->p_level = 1.5f;

    // HEADING HOLD
    drone->p_heading = 1.1f;

    // OPTICAL FLOW CASCADE (Position Hold) - Valeurs par défaut conservatrices
    drone->flow_kp_pos = KP_POS_DEFAULT;          // 0.3 - Gain P position
    drone->flow_kp_vel = KP_VEL_DEFAULT;          // 0.2 - Gain P vélocité
    drone->flow_ki_vel = KI_VEL_DEFAULT;          // 0.0 - COMMENCER A ZERO!
    drone->flow_kd_vel = KD_VEL_DEFAULT;          // 0.0 - COMMENCER A ZERO!
    drone->flow_scale = FLOW_SCALE_DEFAULT;       // 1/800 - Facteur d'échelle
    drone->flow_sign_pitch = FLOW_SIGN_PITCH_DEFAULT;  // 1.0
    drone->flow_sign_roll = FLOW_SIGN_ROLL_DEFAULT;    // 1.0
    drone->flow_vel_max = VEL_TARGET_MAX_DEFAULT; // 1.0 m/s
    drone->flow_angle_max = ANGLE_CMD_MAX_DEFAULT; // 15.0 deg

    // ALTITUDE HOLD
    drone->p_alt = 200.0f;
    drone->i_alt = 50.0f;
    drone->d_alt = 0.0f;
    drone->altitude_target = 0.0f;
    drone->altitude_lock = false;
    drone->pid_throttle_adjust = 0.0f;

    // TRIM MECANIQUE D'ANGLE (ajustable via interface web, en plus de la calibration level)
    drone->trim_roll = 0.0f;
    drone->trim_pitch = 0.0f;

    Serial.println("PID Params Initialized (Flight-Ready Mode + VBat Comp)");
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

    // Reset Heading Hold
    yaw_target_angle = 0.0f;
    yaw_heading_lock = false;
    last_valid_yaw = 0.0f;

    // Reset Altitude Hold
    pid_i_mem_alt = 0.0f;
    pid_last_alt_error = 0.0f;

    // Reset Position Hold - Ces variables sont maintenant dans DroneState
    // Elles seront réinitialisées via pid_reset_position_hold()
}

// --- CALCUL DES CONSIGNES (Setpoints) ---
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

    // ==========================================================================
    // POSITION HOLD - CASCADE COMPLETE (PROBLEMES 3, 4, 5, 6, 7, 8, 9, 10)
    // Architecture: Position → Vélocité → Angle
    // La compensation gyro est faite UNE SEULE FOIS dans flow_compute_velocity()
    // ==========================================================================
    {
        // Détection sticks centrés (PROBLEME 8)
        bool roll_centered  = (drone->channel_1 >= (1500 - STICK_DEADBAND) &&
                               drone->channel_1 <= (1500 + STICK_DEADBAND));
        bool pitch_centered = (drone->channel_2 >= (1500 - STICK_DEADBAND) &&
                               drone->channel_2 <= (1500 + STICK_DEADBAND));
        bool sticks_centered = roll_centered && pitch_centered;

        // Conditions de validité
        bool lidar_ok = (drone->lidar_dist_m >= FLOW_MIN_ALT &&
                         drone->lidar_dist_m <= FLOW_MAX_ALT);
        bool flow_ok  = (drone->flow_quality >= FLOW_QUALITY_MIN &&
                         drone->flow_valid && drone->flow_feature_valid);
        bool in_flight = (drone->current_mode == MODE_FLYING);

        // --- PROBLEME 10: N'intégrer QUE sur nouvelles données flow ---
        if (drone->flow_data_new && flow_ok && lidar_ok && in_flight) {

            // --- ETAPE 1: Intégration de position (PROBLEMES 3, 6) ---
            // On utilise velocity_est_x/y (m/s) déjà compensés par flow_compute_velocity()
            // PAS de double compensation gyro ici (PROBLEME 4)
            drone->position_x += drone->velocity_est_x * drone->dt_flow;
            drone->position_y += drone->velocity_est_y * drone->dt_flow;

            // Anti-windup sur la position (PROBLEME 7: pas de leaky integrator)
            if (drone->position_x > FLOW_POS_CLAMP) drone->position_x = FLOW_POS_CLAMP;
            else if (drone->position_x < -FLOW_POS_CLAMP) drone->position_x = -FLOW_POS_CLAMP;
            if (drone->position_y > FLOW_POS_CLAMP) drone->position_y = FLOW_POS_CLAMP;
            else if (drone->position_y < -FLOW_POS_CLAMP) drone->position_y = -FLOW_POS_CLAMP;
        }

        // --- ETAPE 2: Gestion de l'ancre (PROBLEME 8) ---
        if (sticks_centered && !drone->was_sticks_centered && in_flight) {
            // Transition: sticks viennent d'être relâchés → capturer l'ancre
            drone->anchor_x = drone->position_x;
            drone->anchor_y = drone->position_y;
            // Reset des intégrateurs vélocité pour départ propre
            drone->vel_integral_pitch = 0.0f;
            drone->vel_integral_roll = 0.0f;
            drone->vel_prev_err_pitch = 0.0f;
            drone->vel_prev_err_roll = 0.0f;
        }
        drone->was_sticks_centered = sticks_centered;

        // --- ETAPE 3: Cascade PID si conditions réunies ---
        if (sticks_centered && flow_ok && lidar_ok && in_flight && drone->flow_data_new) {

            // --- NIVEAU 1: Position → Vélocité cible (PROBLEME 5) ---
            float pos_err_x = drone->anchor_x - drone->position_x;
            float pos_err_y = drone->anchor_y - drone->position_y;

            // Utilisation des variables drone->... pour permettre le tuning Wifi
            float vel_target_x = pos_err_x * drone->flow_kp_pos;
            float vel_target_y = pos_err_y * drone->flow_kp_pos;

            // Clamper la vélocité cible
            float v_max = drone->flow_vel_max;
            if (vel_target_x > v_max) vel_target_x = v_max;
            else if (vel_target_x < -v_max) vel_target_x = -v_max;
            if (vel_target_y > v_max) vel_target_y = v_max;
            else if (vel_target_y < -v_max) vel_target_y = -v_max;

            // --- NIVEAU 2: Vélocité → Angle cible (PROBLEME 5) ---
            // PROBLEME 9: Mapping des axes
            // TODO CALIBRATION: vérifier quel axe flow correspond à Pitch/Roll
            // Convention supposée: X_flow = avant/arrière = Pitch, Y_flow = latéral = Roll
            float vel_err_pitch = vel_target_y - drone->velocity_est_y;  // Y = Pitch
            float vel_err_roll  = vel_target_x - drone->velocity_est_x;  // X = Roll

            // PID complet sur la vélocité
            float dt = drone->dt_flow;

            // Pitch
            drone->vel_integral_pitch += vel_err_pitch * dt;
            if (drone->vel_integral_pitch > VEL_INTEGRAL_MAX)
                drone->vel_integral_pitch = VEL_INTEGRAL_MAX;
            else if (drone->vel_integral_pitch < -VEL_INTEGRAL_MAX)
                drone->vel_integral_pitch = -VEL_INTEGRAL_MAX;

            float vel_d_pitch = (vel_err_pitch - drone->vel_prev_err_pitch) / dt;
            drone->vel_prev_err_pitch = vel_err_pitch;

            // Utiliser drone->flow_kp_vel, drone->flow_ki_vel, drone->flow_kd_vel
            float angle_cmd_pitch = drone->flow_kp_vel * vel_err_pitch
                                  + drone->flow_ki_vel * drone->vel_integral_pitch
                                  + drone->flow_kd_vel * vel_d_pitch;

            // Roll
            drone->vel_integral_roll += vel_err_roll * dt;
            if (drone->vel_integral_roll > VEL_INTEGRAL_MAX)
                drone->vel_integral_roll = VEL_INTEGRAL_MAX;
            else if (drone->vel_integral_roll < -VEL_INTEGRAL_MAX)
                drone->vel_integral_roll = -VEL_INTEGRAL_MAX;

            float vel_d_roll = (vel_err_roll - drone->vel_prev_err_roll) / dt;
            drone->vel_prev_err_roll = vel_err_roll;

            float angle_cmd_roll = drone->flow_kp_vel * vel_err_roll
                                 + drone->flow_ki_vel * drone->vel_integral_roll
                                 + drone->flow_kd_vel * vel_d_roll;

            // Clamper les commandes d'angle (utiliser drone->flow_angle_max)
            float a_max = drone->flow_angle_max;
            if (angle_cmd_pitch > a_max) angle_cmd_pitch = a_max;
            else if (angle_cmd_pitch < -a_max) angle_cmd_pitch = -a_max;
            if (angle_cmd_roll > a_max) angle_cmd_roll = a_max;
            else if (angle_cmd_roll < -a_max) angle_cmd_roll = -a_max;

            // --- ETAPE 4: Injection dans les setpoints (PROBLEME 9) ---
            // Les corrections sont en degrés, les setpoints en deg/s via p_level
            // Utiliser drone->flow_sign_pitch/roll pour ajuster orientation
            drone->pid_setpoint_pitch += angle_cmd_pitch * drone->flow_sign_pitch;
            drone->pid_setpoint_roll  += angle_cmd_roll * drone->flow_sign_roll;
        }
        else if (!sticks_centered && in_flight) {
            // Sticks actifs: optionnel, continuer le tracking de position
            // mais ne pas appliquer de correction
            // Reset des intégrateurs pour éviter accumulation pendant pilotage manuel
            drone->vel_integral_pitch = 0.0f;
            drone->vel_integral_roll = 0.0f;
        }
        else if (!in_flight) {
            // Pas en vol: reset complet
            drone->position_x = 0.0f;
            drone->position_y = 0.0f;
            drone->anchor_x = 0.0f;
            drone->anchor_y = 0.0f;
            drone->vel_integral_pitch = 0.0f;
            drone->vel_integral_roll = 0.0f;
            drone->vel_prev_err_pitch = 0.0f;
            drone->vel_prev_err_roll = 0.0f;
        }
    }

    // --- YAW ---
#if HEADING_HOLD_ENABLED
    float raw_stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        raw_stick_yaw = (float)(drone->channel_4 - 1500);
    }

#if RC_INVERT_YAW
    raw_stick_yaw = -raw_stick_yaw;
#endif

    bool stick_in_deadband = (fabsf(raw_stick_yaw) < YAW_DEADBAND);

    if (stick_in_deadband) {
        // --- Heading Hold ---
        float current_yaw = drone->angle_yaw;

        float yaw_delta = current_yaw - last_valid_yaw;
        while (yaw_delta > 180.0f)  yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;

        if (fabsf(yaw_delta) > YAW_JUMP_THRESHOLD && yaw_heading_lock) {
            current_yaw = last_valid_yaw;
        } else {
            last_valid_yaw = current_yaw;
        }

        if (!yaw_heading_lock) {
            yaw_target_angle = current_yaw;
            last_valid_yaw = current_yaw;
            yaw_heading_lock = true;
        }

        float error_angle = yaw_target_angle - current_yaw;
        while (error_angle > 180.0f)  error_angle -= 360.0f;
        while (error_angle < -180.0f) error_angle += 360.0f;

        float rate_setpoint = error_angle * drone->p_heading;

        if (rate_setpoint > YAW_RATE_LIMIT) rate_setpoint = YAW_RATE_LIMIT;
        else if (rate_setpoint < -YAW_RATE_LIMIT) rate_setpoint = -YAW_RATE_LIMIT;

        drone->pid_setpoint_yaw = rate_setpoint;
        ff_sp_yaw = 0.0f;

    } else {
        // --- Pilot Control ---
        yaw_heading_lock = false;
        last_valid_yaw = drone->angle_yaw;

        float stick_yaw = 0.0f;
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);

#if RC_INVERT_YAW
        stick_yaw = -stick_yaw;
#endif

        ff_sp_yaw = stick_yaw;
        drone->pid_setpoint_yaw = stick_yaw;
    }

#else
    // --- YAW SIMPLE ---
    float stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);
    }

#if RC_INVERT_YAW
    stick_yaw = -stick_yaw;
#endif

    ff_sp_yaw = stick_yaw;
    drone->pid_setpoint_yaw = stick_yaw;
#endif
}

// --- BOUCLE PID PRINCIPALE (Avec V-Bat Compensation) ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // 1) DETECTION "IN FLIGHT"
    if (drone->channel_3 > 1300) {
        pid_inflight_timer = millis();
    }
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2) TPA (Throttle PID Attenuation)
    float tpa_factor = 1.0f;
    if (drone->channel_3 > 1500) {
        tpa_factor = map(drone->channel_3, 1500, 2000, 100, 90) / 100.0f;
    }

    // 3) COMPENSATION TENSION BATTERIE (V-Bat)
    float vbat_compensation = 1.0f;
    if (drone->voltage_bat > 6.0f) {
        vbat_compensation = PID_REF_VOLTAGE / drone->voltage_bat;
        if (vbat_compensation > 1.3f) vbat_compensation = 1.3f;
        if (vbat_compensation < 0.7f) vbat_compensation = 0.7f;
    }

    // SCALER GLOBAL
    float pid_scaler = tpa_factor * vbat_compensation;

    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    float p_term_roll = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    
    float d_term_roll = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

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

    float ff_term_roll = (drone->ff_pitch_roll * pid_scaler) * ff_sp_roll;

    drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll + ff_term_roll;
    if (drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if (drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    float p_term_pitch = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;
    
    float d_term_pitch = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

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

    float ff_term_pitch = (drone->ff_pitch_roll * pid_scaler) * ff_sp_pitch;

    drone->pid_output_pitch = p_term_pitch + pid_i_mem_pitch + d_term_pitch + ff_term_pitch;
    if (drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if (drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    float p_term_yaw = (drone->p_yaw * pid_scaler) * error;

    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;
    
    float d_term_yaw = (drone->d_yaw * pid_scaler) * d_err_filtered;

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

    float ff_term_yaw = (drone->ff_yaw * pid_scaler) * ff_sp_yaw;

    drone->pid_output_yaw = p_term_yaw + pid_i_mem_yaw + d_term_yaw + ff_term_yaw;
    if (drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if (drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;

    // ---------------- ALTITUDE HOLD (Désactivé pour l'instant) ----------------
    #if 0 
    // Code Altitude Hold supprimé pour clarté
    #endif 
}