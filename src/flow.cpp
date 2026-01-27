#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"

// --- CONFIGURATION ---
static HardwareSerial FlowSerial(2);

// --- CONSTANTES MSP ---
#define MSP_HEADER_START 0x24 
#define MSP2_SENSOR_RANGEFINDER 0x1F01 
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02 

// Variables globales internes (utilisées par la tâche)
static uint8_t msp_state = 0;
static uint8_t msp_crc = 0;
static uint16_t msp_idx = 0;
static uint16_t msp_payload_size = 0;
static uint16_t msp_cmd_id = 0;
static uint8_t msp_buffer[128];

// Handle de la tâche FreeRTOS
TaskHandle_t FlowTaskHandle = NULL;

// --- CRC8 ---
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

// --- TRAITEMENT DU PAQUET (Fonction interne) ---
void process_packet_v2(DroneState* drone) {
    // LIDAR
    if (msp_cmd_id == MSP2_SENSOR_RANGEFINDER) {
        int32_t dist_mm = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        if (dist_mm > 0 && dist_mm < 5000) {
            drone->lidar_dist_m = dist_mm / 1000.0f;
        } else {
            drone->lidar_dist_m = -1.0f;
        }
    }
    // OPTICAL FLOW
    else if (msp_cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        uint8_t quality = msp_buffer[0];
        int32_t motion_x = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        int32_t motion_y = msp_buffer[5] | (msp_buffer[6] << 8) | (msp_buffer[7] << 16) | (msp_buffer[8] << 24);

        drone->flow_x_rad = motion_x / 10.0f; 
        drone->flow_y_rad = motion_y / 10.0f;
        drone->flow_quality = quality;
        drone->flow_valid = true;
    }
}

// --- LA TÂCHE FREERTOS (Tourne sur le Coeur 0) ---
void FlowTaskCode(void * parameter) {
    DroneState* drone = (DroneState*)parameter;

    for(;;) { // Boucle infinie de la tâche
        
        // Lecture de TOUT ce qui est disponible dans le buffer série
        while (FlowSerial.available()) {
            uint8_t c = FlowSerial.read();

            switch (msp_state) {
                case 0: // IDLE
                    if (c == MSP_HEADER_START) msp_state = 1; 
                    break;
                case 1: // VERSION
                    if (c == 'X') msp_state = 10; // V2
                    else msp_state = 0; 
                    break;
                // V2 PARSER
                case 10: msp_crc = 0; msp_idx = 0; msp_state = 11; break;
                case 11: 
                    msp_buffer[msp_idx++] = c; msp_crc = crc8_dvb_s2(msp_crc, c);
                    if (msp_idx == 5) {
                        msp_cmd_id = msp_buffer[1] | (msp_buffer[2] << 8);
                        msp_payload_size = msp_buffer[3] | (msp_buffer[4] << 8);
                        msp_idx = 0;
                        if (msp_payload_size > 100) msp_state = 0; 
                        else msp_state = (msp_payload_size > 0) ? 12 : 13;
                    }
                    break;
                case 12: 
                    msp_buffer[msp_idx++] = c; msp_crc = crc8_dvb_s2(msp_crc, c);
                    if (msp_idx == msp_payload_size) msp_state = 13; 
                    break;
                case 13: 
                    if (msp_crc == c) process_packet_v2(drone);
                    msp_state = 0; 
                    break;
                default: msp_state = 0; break;
            }
        }

        // IMPORTANT : Laisser respirer le CPU
        // On attend 10ms (soit 100Hz de rafraichissement), ce qui est suffisant pour ce capteur
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- INITIALISATION ---
void flow_init(DroneState* drone) {
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    
    // Création de la tâche sur le COEUR 0 (Core 0)
    // Le vol tourne sur le Core 1, donc aucun ralentissement !
    xTaskCreatePinnedToCore(
        FlowTaskCode,   // Fonction de la tâche
        "FlowTask",     // Nom
        2048,           // Stack size (2ko suffisent)
        drone,          // Paramètre passé (le pointeur drone)
        1,              // Priorité (Basse, le vol est prioritaire)
        &FlowTaskHandle,// Handle
        0               // COEUR 0 (Important !)
    );
    
    Serial.println("FLOW: Tache FreeRTOS demarree sur Core 0");
}

// Cette fonction ne sert plus à rien car la tâche tourne toute seule
// On la garde vide pour compatibilité si elle est encore appelée quelque part
void flow_update(DroneState* drone) {
    // Vide
}