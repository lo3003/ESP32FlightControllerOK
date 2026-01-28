#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"

// --- CONFIGURATION ---
static HardwareSerial FlowSerial(1);

// --- CONSTANTES MSP ---
#define MSP_HEADER_START 0x24 
#define MSP2_SENSOR_RANGEFINDER 0x1F01 
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02 

// Variables globales internes
static uint8_t msp_state = 0;
static uint8_t msp_crc = 0;
static uint16_t msp_idx = 0;
static uint16_t msp_payload_size = 0;
static uint16_t msp_cmd_id = 0;
static uint8_t msp_buffer[128]; // Buffer de décodage suffisant

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

// --- TRAITEMENT DU PAQUET (AVEC LISSAGE + SNAP TO ZERO) ---
void process_packet_v2(DroneState* drone) {
    // LIDAR
    if (msp_cmd_id == MSP2_SENSOR_RANGEFINDER) {
        int32_t dist_mm = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        
        if (dist_mm >= 0 && dist_mm < 5000) { 
            float new_dist = dist_mm / 1000.0f;
            if (drone->lidar_dist_m < 0) drone->lidar_dist_m = new_dist; 
            else drone->lidar_dist_m = (drone->lidar_dist_m * 0.70f) + (new_dist * 0.30f);
        } else {
            drone->lidar_dist_m = -1.0f; 
        }
    }
    // OPTICAL FLOW
    else if (msp_cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        uint8_t quality = msp_buffer[0];
        int32_t motion_x = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        int32_t motion_y = msp_buffer[5] | (msp_buffer[6] << 8) | (msp_buffer[7] << 16) | (msp_buffer[8] << 24);

        // Conversion brute
        float raw_x = motion_x / 10.0f; 
        float raw_y = motion_y / 10.0f;

        // 1. FILTRE PASSE-BAS (Douceur)
        drone->flow_x_rad = (drone->flow_x_rad * 0.60f) + (raw_x * 0.40f);
        drone->flow_y_rad = (drone->flow_y_rad * 0.60f) + (raw_y * 0.40f);

        // 2. SNAP TO ZERO (Propreté)
        // Si le mouvement est négligeable (bruit de fond ou résidu de filtre), on force 0.
        // 0.05 rad/s est un seuil très bas, invisible en vol mais parfait pour le stationnaire.
        if (fabsf(drone->flow_x_rad) < 0.05f) drone->flow_x_rad = 0.0f;
        if (fabsf(drone->flow_y_rad) < 0.05f) drone->flow_y_rad = 0.0f;
        
        drone->flow_quality = quality;
        drone->flow_valid = true;
    }
}

// --- TÂCHE ---
void FlowTaskCode(void * parameter) {
    DroneState* drone = (DroneState*)parameter;

    for(;;) { 
        // Timeout de sécurité : 2ms max de traitement continu
        unsigned long t_start = micros();
        
        while (FlowSerial.available() && (micros() - t_start < 2000)) {
            uint8_t c = FlowSerial.read();
            
            // Machine à états MSP V2
            switch (msp_state) {
                case 0: 
                    if (c == MSP_HEADER_START) msp_state = 1; 
                    else if (c == '$') msp_state = 1; 
                    break;
                case 1: 
                    if (c == 'X') msp_state = 10; 
                    else msp_state = 0; 
                    break;
                case 10: msp_crc = 0; msp_idx = 0; msp_state = 11; break;
                case 11: 
                    msp_buffer[msp_idx++] = c; msp_crc = crc8_dvb_s2(msp_crc, c);
                    if (msp_idx == 5) {
                        msp_cmd_id = msp_buffer[1] | (msp_buffer[2] << 8);
                        msp_payload_size = msp_buffer[3] | (msp_buffer[4] << 8);
                        msp_idx = 0;
                        if (msp_payload_size > 100) msp_state = 0; // Protection overflow
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
        
        // Pause cruciale pour laisser le WiFi travailler sur le Core 0
        vTaskDelay(5 / portTICK_PERIOD_MS); 
    }
}

// --- INIT ---
void flow_init(DroneState* drone) {
    // 1. Buffer raisonnable (1024 suffisent LARGEMENT pour 115200 bauds)
    // 8192 était trop gros et causait probablement une fragmentation mémoire (Freeze)
    FlowSerial.setRxBufferSize(1024); 
    
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    
    // 2. Augmentation de la Stack de la tâche (2048 -> 4096)
    // Pour éviter tout "Stack Overflow" qui planterait l'ESP
    xTaskCreatePinnedToCore(
        FlowTaskCode,   
        "FlowTask",     
        4096,            // Stack doublée pour sécurité
        drone,          
        1,               // Priorité basse (laisser WiFi prioritaire)
        &FlowTaskHandle,
        0                // Core 0
    );
    
    Serial.println("FLOW: Init OK (Buffer 1024 / Stack 4096)");
}

void flow_update(DroneState* drone) {}