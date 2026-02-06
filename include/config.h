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

// =============================================================================
// OPTICAL FLOW - POSITION HOLD CONFIGURATION
// =============================================================================

// --- Calibration Flow - VALEURS PAR DEFAUT pour DroneState ---
// Ces valeurs sont utilisées pour l'initialisation de DroneState.
// En runtime, les gains sont modifiables via la télémétrie.
// Facteur d'échelle counts->rad/s. Convention iNav: 1/800.
#define FLOW_SCALE_DEFAULT   (1.0f / 800.0f)
// Legacy define pour compatibilité (sera remplacé par drone->flow_scale)
#define FLOW_SCALE_FACTOR    FLOW_SCALE_DEFAULT

// --- Seuils de validité ---
#define FLOW_QUALITY_MIN     30      // Qualité minimum capteur (0-255)
#define FLOW_MIN_ALT         0.1f    // Altitude minimum en mètres
#define FLOW_MAX_ALT         4.0f    // Altitude maximum en mètres

// --- Filtre LPF unique sur flow brut ---
#define FLOW_LPF_ALPHA       0.4f    // 0.4 = réactif mais lisse

// --- Limites de sécurité ---
#define FLOW_VEL_CLAMP       3.0f    // Max vitesse estimée m/s
#define FLOW_POS_CLAMP       5.0f    // Max position accumulée m (anti-windup)
#define FLOW_DT_MIN          0.01f   // dt minimum entre paquets (100 Hz max)
#define FLOW_DT_MAX          0.5f    // dt maximum (timeout si > 0.5s)

// --- Mapping axes - VALEURS PAR DEFAUT (modifiables via télémétrie) ---
// Si flèche Matek pointe vers l'avant du drone:
//   X_capteur = déplacement avant/arrière = Pitch
//   Y_capteur = déplacement latéral = Roll
// Inverser les signes si le drone corrige dans le mauvais sens.
#define FLOW_SIGN_PITCH_DEFAULT  1.0f    // 1.0 ou -1.0 selon orientation
#define FLOW_SIGN_ROLL_DEFAULT   1.0f    // 1.0 ou -1.0 selon orientation
// Legacy defines pour compatibilité
#define FLOW_SIGN_PITCH      FLOW_SIGN_PITCH_DEFAULT
#define FLOW_SIGN_ROLL       FLOW_SIGN_ROLL_DEFAULT

// --- PID Position - VALEURS PAR DEFAUT (modifiables via télémétrie) ---
#define KP_POS_DEFAULT       0.3f    // Gain P position (réduit pour sécurité)
#define VEL_TARGET_MAX_DEFAULT 1.0f  // Vitesse cible max m/s
// Legacy defines
#define KP_POS               KP_POS_DEFAULT
#define VEL_TARGET_MAX       VEL_TARGET_MAX_DEFAULT

// --- PID Vélocité - VALEURS PAR DEFAUT (modifiables via télémétrie) ---
#define KP_VEL_DEFAULT       0.2f    // Gain P vélocité (réduit pour sécurité)
#define KI_VEL_DEFAULT       0.0f    // COMMENCER A ZERO! Augmenter progressivement
#define KD_VEL_DEFAULT       0.0f    // COMMENCER A ZERO! Augmenter progressivement
#define ANGLE_CMD_MAX_DEFAULT 15.0f  // Max correction angle en degrés
#define VEL_INTEGRAL_MAX     5.0f    // Anti-windup intégrateur vélocité (fixe)
// Legacy defines
#define KP_VEL               KP_VEL_DEFAULT
#define KI_VEL               KI_VEL_DEFAULT
#define KD_VEL               KD_VEL_DEFAULT
#define ANGLE_CMD_MAX        ANGLE_CMD_MAX_DEFAULT

// --- Deadband sticks (pour détecter "sticks centrés") ---
#define STICK_DEADBAND       50      // ±50 autour de 1500 (1450-1550)

#endif