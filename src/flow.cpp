/**
 * @file flow.cpp
 * @brief Driver MSP v2 robuste pour MatekSys 3901-L0X (Optical Flow + Lidar)
 *
 * Protocole MSP v2:
 *   Header: '$' 'X' '<' (direction: from sensor)
 *   Flag:   1 byte (0 = request, autres = reserved)
 *   Cmd:    2 bytes (little-endian) - Function ID
 *   Size:   2 bytes (little-endian) - Payload size
 *   Payload: [Size] bytes
 *   CRC:    1 byte (CRC8 DVB-S2 sur Flag + Cmd + Size + Payload)
 *
 * IDs supportés:
 *   MSP2_SENSOR_RANGEFINDER (0x1F01): Distance Lidar
 *   MSP2_SENSOR_OPTIC_FLOW  (0x1F02): Données Optical Flow
 */

#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =============================================================================
// CONSTANTES MSP V2
// =============================================================================
#define MSP_HEADER_DOLLAR   '$'
#define MSP_HEADER_X        'X'
#define MSP_DIRECTION_FROM  '<'   // Sensor -> FC (nous recevons)
#define MSP_DIRECTION_TO    '>'   // FC -> Sensor (non utilisé ici)

#define MSP2_SENSOR_RANGEFINDER 0x1F01
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02

#define MSP_MAX_PAYLOAD_SIZE    64
#define MSP_TIMEOUT_MS          200   // Timeout pour invalider les données

// Seuil de qualité pour considérer les features comme valides
#define FLOW_QUALITY_THRESHOLD  50

// =============================================================================
// MACHINE À ÉTATS MSP V2
// =============================================================================
typedef enum {
    MSP_STATE_IDLE = 0,         // Attente '$'
    MSP_STATE_HEADER_X,         // Reçu '$', attente 'X'
    MSP_STATE_HEADER_DIR,       // Reçu 'X', attente '<'
    MSP_STATE_FLAG,             // Lecture Flag
    MSP_STATE_CMD_LOW,          // Lecture Cmd byte low
    MSP_STATE_CMD_HIGH,         // Lecture Cmd byte high
    MSP_STATE_SIZE_LOW,         // Lecture Size byte low
    MSP_STATE_SIZE_HIGH,        // Lecture Size byte high
    MSP_STATE_PAYLOAD,          // Lecture Payload
    MSP_STATE_CRC               // Vérification CRC
} MspState_t;

// =============================================================================
// STRUCTURES DE DONNÉES
// =============================================================================

// Snapshot thread-safe pour transfert Core0 -> Core1
typedef struct {
    float flow_raw_x;           // rad/s brut
    float flow_raw_y;           // rad/s brut
    uint8_t quality;            // 0-255
    float lidar_dist_m;         // mètres (-1 si invalide)
    bool data_valid;            // Capteur répond
    uint32_t last_update_ms;    // Timestamp dernière mise à jour
} FlowSnapshot_t;

// Contexte du parser MSP
typedef struct {
    MspState_t state;
    uint8_t crc;
    uint8_t flag;
    uint16_t cmd_id;
    uint16_t payload_size;
    uint16_t payload_idx;
    uint8_t payload[MSP_MAX_PAYLOAD_SIZE];
} MspParser_t;

// =============================================================================
// VARIABLES STATIQUES
// =============================================================================
static HardwareSerial FlowSerial(1);    // UART1
static TaskHandle_t FlowTaskHandle = NULL;

// Snapshot protégé par spinlock
static FlowSnapshot_t g_flow_snapshot = {0.0f, 0.0f, 0, -1.0f, false, 0};
static portMUX_TYPE g_flow_mux = portMUX_INITIALIZER_UNLOCKED;

// Parser MSP (local à la tâche)
static MspParser_t g_parser = {MSP_STATE_IDLE, 0, 0, 0, 0, 0, {0}};

// Variables intermédiaires avec filtrage LPF
static float s_flow_x_filtered = 0.0f;
static float s_flow_y_filtered = 0.0f;
static float s_lidar_filtered = -1.0f;

// =============================================================================
// CRC8 DVB-S2
// =============================================================================
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// =============================================================================
// TRAITEMENT DES PAQUETS MSP DÉCODÉS
// =============================================================================
static void process_msp_packet(MspParser_t* parser) {
    uint32_t now_ms = millis();

    if (parser->cmd_id == MSP2_SENSOR_RANGEFINDER) {
        // Payload Rangefinder (5 bytes minimum):
        //   [0]: Quality (0-255)
        //   [1-4]: Distance en mm (int32_t little-endian)
        if (parser->payload_size >= 5) {
            // uint8_t range_quality = parser->payload[0]; // Non utilisé pour l'instant
            int32_t dist_mm = (int32_t)(
                parser->payload[1] |
                (parser->payload[2] << 8) |
                (parser->payload[3] << 16) |
                (parser->payload[4] << 24)
            );

            // Validation: 0 - 5000mm (5m max pour VL53L0X)
            if (dist_mm >= 0 && dist_mm < 5000) {
                float new_dist = dist_mm / 1000.0f;
                // Filtre LPF: 70% ancien + 30% nouveau
                if (s_lidar_filtered < 0.0f) {
                    s_lidar_filtered = new_dist;
                } else {
                    s_lidar_filtered = (s_lidar_filtered * 0.7f) + (new_dist * 0.3f);
                }
            } else {
                s_lidar_filtered = -1.0f;
            }

            // Mise à jour snapshot (section critique courte)
            portENTER_CRITICAL(&g_flow_mux);
            g_flow_snapshot.lidar_dist_m = s_lidar_filtered;
            g_flow_snapshot.last_update_ms = now_ms;
            g_flow_snapshot.data_valid = true;
            portEXIT_CRITICAL(&g_flow_mux);
        }
    }
    else if (parser->cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        // Payload Optic Flow (9 bytes):
        //   [0]: Quality (0-255)
        //   [1-4]: motion_x (int32_t, motion count)
        //   [5-8]: motion_y (int32_t, motion count)
        //
        // Note: Les valeurs sont des "counts" de mouvement accumulés.
        // On les convertit en rad/s approximatif.
        // Le scaling dépend du capteur et de la fréquence de lecture.
        if (parser->payload_size >= 9) {
            uint8_t quality = parser->payload[0];

            int32_t motion_x = (int32_t)(
                parser->payload[1] |
                (parser->payload[2] << 8) |
                (parser->payload[3] << 16) |
                (parser->payload[4] << 24)
            );

            int32_t motion_y = (int32_t)(
                parser->payload[5] |
                (parser->payload[6] << 8) |
                (parser->payload[7] << 16) |
                (parser->payload[8] << 24)
            );

            // Conversion counts -> rad/s approximatif
            // Le PMW3901 a ~35 counts/deg à 1m de hauteur
            // À ajuster selon les specs exactes du Matek
            const float COUNTS_TO_RAD = 1.0f / (35.0f * 57.2958f); // deg->rad

            float raw_x = (float)motion_x * COUNTS_TO_RAD;
            float raw_y = (float)motion_y * COUNTS_TO_RAD;

            // Filtre LPF réactif: 50% ancien + 50% nouveau
            s_flow_x_filtered = (s_flow_x_filtered * 0.5f) + (raw_x * 0.5f);
            s_flow_y_filtered = (s_flow_y_filtered * 0.5f) + (raw_y * 0.5f);

            // Deadband fine pour bruit statique
            if (fabsf(s_flow_x_filtered) < 0.001f) s_flow_x_filtered = 0.0f;
            if (fabsf(s_flow_y_filtered) < 0.001f) s_flow_y_filtered = 0.0f;

            // Mise à jour snapshot
            portENTER_CRITICAL(&g_flow_mux);
            g_flow_snapshot.flow_raw_x = s_flow_x_filtered;
            g_flow_snapshot.flow_raw_y = s_flow_y_filtered;
            g_flow_snapshot.quality = quality;
            g_flow_snapshot.last_update_ms = now_ms;
            g_flow_snapshot.data_valid = (quality > FLOW_QUALITY_THRESHOLD);
            portEXIT_CRITICAL(&g_flow_mux);
        }
    }
    // Autres CMD_IDs ignorés silencieusement
}

// =============================================================================
// PARSER MSP V2 - MACHINE À ÉTATS
// =============================================================================
static void parse_msp_byte(MspParser_t* parser, uint8_t byte) {
    switch (parser->state) {
        case MSP_STATE_IDLE:
            if (byte == MSP_HEADER_DOLLAR) {
                parser->state = MSP_STATE_HEADER_X;
            }
            break;

        case MSP_STATE_HEADER_X:
            if (byte == MSP_HEADER_X) {
                parser->state = MSP_STATE_HEADER_DIR;
            } else {
                parser->state = MSP_STATE_IDLE;
            }
            break;

        case MSP_STATE_HEADER_DIR:
            if (byte == MSP_DIRECTION_FROM) {
                // Début de trame valide, reset CRC
                parser->crc = 0;
                parser->state = MSP_STATE_FLAG;
            } else {
                parser->state = MSP_STATE_IDLE;
            }
            break;

        case MSP_STATE_FLAG:
            parser->flag = byte;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_CMD_LOW;
            break;

        case MSP_STATE_CMD_LOW:
            parser->cmd_id = byte;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_CMD_HIGH;
            break;

        case MSP_STATE_CMD_HIGH:
            parser->cmd_id |= (uint16_t)byte << 8;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_SIZE_LOW;
            break;

        case MSP_STATE_SIZE_LOW:
            parser->payload_size = byte;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_SIZE_HIGH;
            break;

        case MSP_STATE_SIZE_HIGH:
            parser->payload_size |= (uint16_t)byte << 8;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            parser->payload_idx = 0;

            if (parser->payload_size > MSP_MAX_PAYLOAD_SIZE) {
                // Payload trop grand, abandon
                parser->state = MSP_STATE_IDLE;
            } else if (parser->payload_size == 0) {
                // Pas de payload, passer directement au CRC
                parser->state = MSP_STATE_CRC;
            } else {
                parser->state = MSP_STATE_PAYLOAD;
            }
            break;

        case MSP_STATE_PAYLOAD:
            parser->payload[parser->payload_idx++] = byte;
            parser->crc = crc8_dvb_s2(parser->crc, byte);
            if (parser->payload_idx >= parser->payload_size) {
                parser->state = MSP_STATE_CRC;
            }
            break;

        case MSP_STATE_CRC:
            if (parser->crc == byte) {
                // CRC OK, traiter le paquet
                process_msp_packet(parser);
            }
            // Reset pour prochain paquet
            parser->state = MSP_STATE_IDLE;
            break;

        default:
            parser->state = MSP_STATE_IDLE;
            break;
    }
}

// =============================================================================
// TÂCHE FREERTOS (CORE 0)
// =============================================================================
static void FlowTaskCode(void* parameter) {
    (void)parameter;

    for (;;) {
        // Lecture non-bloquante par paquets
        int bytes_available = FlowSerial.available();
        if (bytes_available > 0) {
            // Limiter pour ne pas bloquer trop longtemps
            int to_read = (bytes_available > 64) ? 64 : bytes_available;
            for (int i = 0; i < to_read; i++) {
                uint8_t byte = FlowSerial.read();
                parse_msp_byte(&g_parser, byte);
            }
        }

        // Yield pour autres tâches (5ms = ~200Hz de polling)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =============================================================================
// API PUBLIQUE
// =============================================================================

void flow_init(DroneState* drone) {
    // Initialisation des valeurs par défaut
    drone->lidar_dist_m = -1.0f;
    drone->flow_x_rad = 0.0f;
    drone->flow_y_rad = 0.0f;
    drone->flow_quality = 0;
    drone->flow_valid = false;
    drone->flow_raw_rad_x = 0.0f;
    drone->flow_raw_rad_y = 0.0f;
    drone->velocity_est_x = 0.0f;
    drone->velocity_est_y = 0.0f;
    drone->flow_feature_valid = false;

    // Configuration UART
    FlowSerial.setRxBufferSize(512);
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);

    // Lancement de la tâche sur Core 0 (Core 1 = loop principale)
    xTaskCreatePinnedToCore(
        FlowTaskCode,
        "FlowTask",
        4096,           // Stack size
        NULL,           // Paramètre (non utilisé)
        2,              // Priorité (2 = au-dessus de idle)
        &FlowTaskHandle,
        0               // Core 0
    );

    Serial.println("FLOW: MSP v2 Driver initialized (Core 0)");
}

void flow_update(DroneState* drone) {
    FlowSnapshot_t snapshot;

    // Copie atomique du snapshot
    portENTER_CRITICAL(&g_flow_mux);
    snapshot = g_flow_snapshot;
    portEXIT_CRITICAL(&g_flow_mux);

    // Vérification timeout
    uint32_t now_ms = millis();
    if ((now_ms - snapshot.last_update_ms) > MSP_TIMEOUT_MS) {
        // Données périmées
        drone->flow_valid = false;
        drone->flow_feature_valid = false;
        drone->flow_quality = 0;
        return;
    }

    // Transfert vers DroneState
    drone->flow_x_rad = snapshot.flow_raw_x;
    drone->flow_y_rad = snapshot.flow_raw_y;
    drone->flow_raw_rad_x = snapshot.flow_raw_x;
    drone->flow_raw_rad_y = snapshot.flow_raw_y;
    drone->flow_quality = snapshot.quality;
    drone->lidar_dist_m = snapshot.lidar_dist_m;
    drone->flow_valid = snapshot.data_valid;
    drone->flow_feature_valid = (snapshot.quality > FLOW_QUALITY_THRESHOLD);
}

void flow_compute_velocity(DroneState* drone) {
    // Vérifications de sécurité
    if (!drone->flow_valid || !drone->flow_feature_valid) {
        drone->velocity_est_x = 0.0f;
        drone->velocity_est_y = 0.0f;
        return;
    }

    // Altitude valide requise (Lidar > 5cm et < 4m)
    float altitude = drone->lidar_dist_m;
    if (altitude < 0.05f || altitude > 4.0f) {
        drone->velocity_est_x = 0.0f;
        drone->velocity_est_y = 0.0f;
        return;
    }

    // Conversion gyro deg/s -> rad/s
    // Note: Les axes du gyro et du flow doivent être alignés
    // gyro_roll_input  -> rotation autour de X (affecte flow Y)
    // gyro_pitch_input -> rotation autour de Y (affecte flow X)
    const float deg_to_rad_conv = 0.017453292f;

    float gyro_x_rad = drone->gyro_pitch_input * deg_to_rad_conv;  // Pitch affecte flow X
    float gyro_y_rad = drone->gyro_roll_input * deg_to_rad_conv;   // Roll affecte flow Y

    // Fusion: Vitesse = (Flow_rad/s - Gyro_rad/s) * Altitude_m
    // Cette formule compense le mouvement apparent dû à la rotation du drone
    float vel_x = (drone->flow_raw_rad_x - gyro_x_rad) * altitude;
    float vel_y = (drone->flow_raw_rad_y - gyro_y_rad) * altitude;

    // Limiter les vitesses aberrantes (max 5 m/s)
    const float MAX_VELOCITY = 5.0f;
    if (fabsf(vel_x) > MAX_VELOCITY) vel_x = (vel_x > 0) ? MAX_VELOCITY : -MAX_VELOCITY;
    if (fabsf(vel_y) > MAX_VELOCITY) vel_y = (vel_y > 0) ? MAX_VELOCITY : -MAX_VELOCITY;

    // Filtre LPF léger sur la sortie (80% ancien + 20% nouveau)
    drone->velocity_est_x = (drone->velocity_est_x * 0.8f) + (vel_x * 0.2f);
    drone->velocity_est_y = (drone->velocity_est_y * 0.8f) + (vel_y * 0.2f);
}
