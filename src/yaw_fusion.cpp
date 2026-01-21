#include <Arduino.h>
#include "yaw_fusion.h"

// ==================== CONFIGURATION ====================
// Coefficient du filtre complémentaire
// alpha = 0.98 : 98% gyro (réactif, court terme), 2% magnétomètre (stable, long terme)
#define YAW_FUSION_ALPHA 0.98f

// ==================== VARIABLES D'ETAT ====================
static float fused_yaw = 0.0f;      // Angle yaw fusionné (0-360°)
static bool fusion_initialized = false;

// ==================== FONCTIONS UTILITAIRES ====================

/**
 * Normalise un angle entre 0 et 360 degrés
 */
static float normalize_angle_360(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * Calcule la différence angulaire en tenant compte du wrap-around
 * Retourne une valeur entre -180 et +180
 */
static float angle_diff(float target, float current) {
    float diff = target - current;

    // Gérer le wrap-around
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    return diff;
}

// ==================== API PUBLIQUE ====================

void yaw_fusion_init() {
    fused_yaw = 0.0f;
    fusion_initialized = false;
}

void yaw_fusion_update(DroneState *drone, float dt_s) {
    // Vérifier que le delta time est valide
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = 0.004f;  // Valeur par défaut (250Hz)
    }

    // Récupérer les entrées
    float gyro_rate = drone->gyro_yaw_input;  // deg/s du MPU6050
    float mag_heading = drone->alt_angle_yaw; // Heading magnétomètre (0-360°)

    // Première exécution : initialiser avec le magnétomètre
    if (!fusion_initialized) {
        fused_yaw = mag_heading;
        fusion_initialized = true;
        drone->angle_yaw = fused_yaw;
        return;
    }

    // ========== FILTRE COMPLEMENTAIRE ==========

    // 1. Prédiction par intégration gyro (court terme)
    float gyro_prediction = fused_yaw + gyro_rate * dt_s;
    gyro_prediction = normalize_angle_360(gyro_prediction);

    // 2. Correction par magnétomètre (long terme)
    // Calculer l'erreur angulaire entre mag et prédiction gyro
    float error = angle_diff(mag_heading, gyro_prediction);

    // 3. Fusion : alpha * gyro + (1-alpha) * correction magnétomètre
    // Équivalent à : gyro_prediction + (1-alpha) * error
    fused_yaw = gyro_prediction + (1.0f - YAW_FUSION_ALPHA) * error;

    // Normaliser le résultat
    fused_yaw = normalize_angle_360(fused_yaw);

    // Stocker dans le drone state
    drone->angle_yaw = fused_yaw;
}

void yaw_fusion_reset(DroneState *drone) {
    // Réinitialiser avec la valeur actuelle du magnétomètre
    fused_yaw = drone->alt_angle_yaw;
    fusion_initialized = true;
    drone->angle_yaw = fused_yaw;
}
