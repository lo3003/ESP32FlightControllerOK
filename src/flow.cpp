#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"

// --- CONFIGURATION ---
static HardwareSerial FlowSerial(2); // UART 2 sur pins 16/17

// --- CONSTANTES MSP V2 ---
#define MSP_HEADER_START 0x24 // '$'
#define MSP_HEADER_X     0x58 // 'X' (V2)
#define MSP_HEADER_M     0x4D // 'M' (V1 - Support Legacy)

// Nouveaux IDs découverts lors de vos tests
#define MSP2_SENSOR_RANGEFINDER 0x1F01 // Lidar
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02 // Optical Flow

// Variables internes
static uint8_t msp_state = 0;
static uint8_t msp_crc = 0;
static uint16_t msp_idx = 0;
static uint16_t msp_payload_size = 0;
static uint16_t msp_cmd_id = 0;
static uint8_t msp_buffer[128];

// --- CRC8 pour MSP V2 ---
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

void flow_init() {
    // Initialisation avec les pins validées (16 et 17)
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    // Note: FLOW_BAUD doit être 115200 dans config.h ou flow.h
}

void process_packet_v2(DroneState* drone) {
    // 1. LIDAR (ID 0x1F01)
    if (msp_cmd_id == MSP2_SENSOR_RANGEFINDER) {
        // Structure: [Qualité(1)] + [Distance mm (4, int32)]
        // uint8_t quality = msp_buffer[0]; 
        int32_t dist_mm = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);

        if (dist_mm > 0 && dist_mm < 5000) {
            drone->lidar_dist_m = dist_mm / 1000.0f; // Conversion mm -> mètres
        } else {
            drone->lidar_dist_m = -1.0f; // Hors de portée
        }
    }
    
    // 2. OPTICAL FLOW (ID 0x1F02)
    else if (msp_cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        // Structure: [Qualité(1)] + [MotionX (4)] + [MotionY (4)]
        uint8_t quality = msp_buffer[0];
        int32_t motion_x = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        int32_t motion_y = msp_buffer[5] | (msp_buffer[6] << 8) | (msp_buffer[7] << 16) | (msp_buffer[8] << 24);

        // Mise à jour de l'état du drone
        // Note: On divise par 10.0f pour garder l'échelle standard, à ajuster selon le vol
        drone->flow_x_rad = motion_x / 10.0f; 
        drone->flow_y_rad = motion_y / 10.0f;
        drone->flow_quality = quality;
        drone->flow_valid = true;
    }
}

void flow_update(DroneState* drone) {
    // Lecture limitée pour ne jamais bloquer la boucle principale (Anti-Freeze)
    int max_bytes = 64;

    while (FlowSerial.available() && max_bytes > 0) {
        uint8_t c = FlowSerial.read();
        max_bytes--;

        switch (msp_state) {
            case 0: // IDLE
                if (c == MSP_HEADER_START) msp_state = 1; 
                break;
            
            case 1: // VERSION
                if (c == 'X') { // V2 (Celle de votre capteur)
                    msp_state = 10; 
                } else if (c == 'M') { // V1 (Legacy, au cas où)
                    msp_state = 0; // On ignore V1 pour simplifier, vu que votre capteur est V2
                } else {
                    msp_state = 0;
                }
                break;

            // --- DECODAGE V2 ---
            case 10: // TYPE (<, >, !)
                msp_crc = 0; 
                msp_idx = 0; 
                msp_state = 11; 
                break;

            case 11: // HEADER (Flag + Cmd + Size)
                msp_buffer[msp_idx++] = c; 
                msp_crc = crc8_dvb_s2(msp_crc, c);
                
                if (msp_idx == 5) {
                    // Extraction Cmd et Size
                    msp_cmd_id = msp_buffer[1] | (msp_buffer[2] << 8);
                    msp_payload_size = msp_buffer[3] | (msp_buffer[4] << 8);
                    
                    msp_idx = 0;
                    // Protection contre les payloads géants
                    if (msp_payload_size > 100) {
                        msp_state = 0; 
                    } else {
                        msp_state = (msp_payload_size > 0) ? 12 : 13;
                    }
                }
                break;

            case 12: // PAYLOAD
                msp_buffer[msp_idx++] = c; 
                msp_crc = crc8_dvb_s2(msp_crc, c);
                if (msp_idx == msp_payload_size) msp_state = 13; 
                break;

            case 13: // CHECKSUM
                if (msp_crc == c) {
                    process_packet_v2(drone);
                }
                msp_state = 0; 
                break;
                
            default:
                msp_state = 0;
                break;
        }
    }

    // --- SECURITE LIDAR (Optionnelle) ---
    // Si le Lidar ne capte rien (trop haut ou bug), on garde une valeur saine pour éviter les divisions par zéro
    // (À retirer une fois en vol réel si vous voulez que le PosHold se désactive sans Lidar)
    if (drone->lidar_dist_m <= 0.0f) {
         drone->lidar_dist_m = 1.0f; // Décommentez pour forcer 1m en cas d'erreur
    }
}