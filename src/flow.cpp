// src/flow.cpp
#include <Arduino.h>
#include "flow.h"
#include "config.h"

// Définitions du protocole MSP
#define MSP_HEADER_START 0x24 // '$'
#define MSP_HEADER_M     0x4D // 'M'
#define MSP_HEADER_ARROW 0x3E // '>'
#define MSP_OPFLOW       105  // ID Optical Flow
#define MSP_RANGEFINDER  119  // ID Lidar

// Instance du port série (Serial1)
static HardwareSerial FlowSerial(1);

// Variables pour le décodage
static uint8_t msp_state = 0;
static uint8_t msp_msg_size = 0;
static uint8_t msp_msg_type = 0;
static uint8_t msp_crc = 0;
static uint8_t msp_buffer[64];
static uint8_t msp_idx = 0;

void flow_init() {
    // Initialisation du Serial1 sur les pins 16/17
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    Serial.println("FLOW: Init Serial1 OK");
}

void process_msp_packet(DroneState* drone) {
    if (msp_msg_type == MSP_OPFLOW) {
        // Le Matek envoie Motion X, Motion Y et Qualité
        int16_t motion_x = (int16_t)(msp_buffer[0] | (msp_buffer[1] << 8));
        int16_t motion_y = (int16_t)(msp_buffer[2] | (msp_buffer[3] << 8));
        uint8_t qual     = msp_buffer[4];

        // Conversion brute pour l'instant
        drone->flow_x_rad = motion_x / 10.0f; 
        drone->flow_y_rad = motion_y / 10.0f;
        drone->flow_quality = qual;
        drone->flow_valid = true;

    } else if (msp_msg_type == MSP_RANGEFINDER) {
        // Le Matek envoie la distance en mm
        int16_t dist_mm = (int16_t)(msp_buffer[0] | (msp_buffer[1] << 8));
        
        if (dist_mm > 0 && dist_mm < 2000) { // Max 2m pour le VL53L0X
             drone->lidar_dist_m = dist_mm / 1000.0f; // Conversion en mètres
        } else {
             drone->lidar_dist_m = -1.0f; // Hors de portée
        }
    }
}

void flow_update(DroneState* drone) {
    // Lecture de tout ce qui est arrivé sur le port série
    while (FlowSerial.available()) {
        uint8_t c = FlowSerial.read();

        // Machine à état pour décoder le MSP ($M>...)
        switch (msp_state) {
            case 0: if (c == '$') msp_state++; break;
            case 1: if (c == 'M') msp_state++; else msp_state = 0; break;
            case 2: if (c == '>') msp_state++; else msp_state = 0; break;
            case 3: // Taille du message
                msp_msg_size = c;
                if (msp_msg_size > 60) msp_state = 0; // Sécurité
                else {
                    msp_crc = c; // Init CRC
                    msp_state++;
                }
                break;
            case 4: // Type de message (105 ou 119)
                msp_msg_type = c;
                msp_crc ^= c;
                msp_idx = 0;
                msp_state++;
                break;
            case 5: // Lecture des données (Payload)
                if (msp_idx < msp_msg_size) {
                    msp_buffer[msp_idx++] = c;
                    msp_crc ^= c;
                }
                if (msp_idx == msp_msg_size) {
                    msp_state++;
                }
                break;
            case 6: // Vérification CRC
                if (msp_crc == c) {
                    process_msp_packet(drone);
                }
                msp_state = 0; // Reset pour le prochain paquet
                break;
            default:
                msp_state = 0;
                break;
        }
    }
}