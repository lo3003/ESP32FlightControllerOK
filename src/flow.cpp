// src/flow.cpp
#include <Arduino.h>
#include "flow.h"
#include "config.h"

// --- CONFIGURATION ---
// Utiliser UART 2 pour les pins 16/17 (Plus stable que UART 1)
static HardwareSerial FlowSerial(2);

// Décommentez pour voir les données brutes dans le moniteur série USB (Attention au spam)
// #define DEBUG_FLOW_RAW 

#define MSP_HEADER_START 0x24 // '$'
#define MSP_OPFLOW       105  // ID Optical Flow
#define MSP_RANGEFINDER  119  // ID Lidar

// Variables internes
static uint8_t msp_state = 0;
static uint8_t msp_msg_size = 0;
static uint8_t msp_msg_type = 0;
static uint8_t msp_crc = 0;
static uint8_t msp_buffer[64];
static uint8_t msp_idx = 0;

void flow_init() {
    // Note: Serial2 est le hardware natif pour 16/17
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    Serial.println("FLOW: Init Optical Flow sur Serial2 (Pins 16/17)");
}

void process_msp_packet(DroneState* drone) {
    if (msp_msg_type == MSP_OPFLOW) {
        // Le Matek envoie Motion X, Motion Y (int16) et Qualité (uint8)
        int16_t motion_x = (int16_t)(msp_buffer[0] | (msp_buffer[1] << 8));
        int16_t motion_y = (int16_t)(msp_buffer[2] | (msp_buffer[3] << 8));
        uint8_t qual     = msp_buffer[4];

        // Stockage dans l'état global
        // Note: Ce sont des "raw pixels" ou "deci-pixels", pas encore des radians/sec
        // Il faudra les convertir avec la distance plus tard
        drone->flow_x_rad = motion_x / 10.0f; 
        drone->flow_y_rad = motion_y / 10.0f;
        drone->flow_quality = qual;
        drone->flow_valid = true;

        #ifdef DEBUG_FLOW_RAW
        Serial.printf("FLOW: X=%d Y=%d Q=%d\n", motion_x, motion_y, qual);
        #endif

    } else if (msp_msg_type == MSP_RANGEFINDER) {
        // Distance en mm
        int16_t dist_mm = (int16_t)(msp_buffer[0] | (msp_buffer[1] << 8));
        
        // Si vous voulez tester SANS lidar réel, on peut tricher ici :
        // if (dist_mm < 0) dist_mm = 1000; // Simule 1m de hauteur

        if (dist_mm > 0 && dist_mm < 2000) { 
             drone->lidar_dist_m = dist_mm / 1000.0f; 
        } else {
             drone->lidar_dist_m = -1.0f; // Trop loin ou erreur
        }
    }
}

void flow_update(DroneState* drone) {
    // Lecture en boucle
    while (FlowSerial.available()) {
        uint8_t c = FlowSerial.read();

        // Machine à état MSP v1
        switch (msp_state) {
            case 0: // Attend '$'
                if (c == '$') msp_state++; 
                else {
                    // Si on reçoit des données mais pas de $, c'est peut-être un mauvais baudrate
                    // ou du bruit.
                }
                break;
            case 1: // Attend 'M'
                if (c == 'M') msp_state++; else msp_state = 0; break;
            case 2: // Attend '>'
                if (c == '>') msp_state++; else msp_state = 0; break;
            case 3: // Taille Payload
                msp_msg_size = c;
                if (msp_msg_size > 60) msp_state = 0; // Protection buffer
                else {
                    msp_crc = c; // CRC commence avec la taille
                    msp_state++;
                }
                break;
            case 4: // Type Message
                msp_msg_type = c;
                msp_crc ^= c;
                msp_idx = 0;
                msp_state++;
                break;
            case 5: // Payload
                if (msp_idx < msp_msg_size) {
                    msp_buffer[msp_idx++] = c;
                    msp_crc ^= c;
                }
                if (msp_idx == msp_msg_size) {
                    msp_state++;
                }
                break;
            case 6: // Verification CRC
                if (msp_crc == c) {
                    process_msp_packet(drone);
                } else {
                    // Erreur CRC -> Debug optionnel
                    // Serial.println("FLOW: CRC Error");
                }
                msp_state = 0; 
                break;
            default:
                msp_state = 0;
                break;
        }
    }
}