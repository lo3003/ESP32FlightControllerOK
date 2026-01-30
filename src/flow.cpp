#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"

// --- CONFIGURATION ---
static HardwareSerial FlowSerial(1); // UART1

// --- CONSTANTES MSP V2 ---
#define MSP_HEADER_START '$'
#define MSP_V2_FRAME_ID  'X'
#define MSP_DIRECTION_RX '<' // From Sensor to FC

// IDs Capteurs Matek 3901-L0X
#define MSP2_SENSOR_RANGEFINDER 0x1F01 
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02 

// --- ETATS DU PARSER MSP (Machine à états ArduPilot simplifiée) ---
enum mspState_e {
    MSP_IDLE,
    MSP_HEADER_START_RECEIVED,
    MSP_HEADER_X_RECEIVED,
    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE
};

// Variables statiques du parser
static mspState_e msp_state = MSP_IDLE;
static uint8_t msp_crc = 0;
static uint16_t msp_idx = 0;
static uint16_t msp_payload_size = 0;
static uint16_t msp_cmd_id = 0;
static uint8_t msp_buffer[128]; // Buffer de réception

// Handle FreeRTOS
TaskHandle_t FlowTaskHandle = NULL;

// --- CRC8 (Polynome DVB-S2 0xD5) ---
// Identique à ArduPilot / Betaflight
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

// --- VARIABLES CALCUL FLUX ---
static unsigned long prev_flow_time = 0;
// Echelle pour Matek 3901-L0X (FOV 42°)
// Note: ArduPilot utilise parfois 700 ou adapte selon la hauteur. 700 est un bon départ.
#define OPTICAL_FLOW_SCALER 700.0f 

// --- TRAITEMENT DES DONNEES VALIDES ---
void process_msp_packet(DroneState* drone) {
    unsigned long now = micros();

    // === LIDAR (Rangefinder) ===
    if (msp_cmd_id == MSP2_SENSOR_RANGEFINDER) {
        // Payload: [Quality(1)][DistL(1)][DistM(1)][DistH(1)][DistHH(1)]
        // Matek envoie la distance en mm (int32)
        // Attention : Buffer[0] est la qualité
        
        int32_t dist_mm = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        // uint8_t quality = msp_buffer[0]; // Si besoin

        // Filtrage simple et conversion
        if (dist_mm >= 0 && dist_mm < 5000) { // Max 5m pour ce capteur
            float new_dist = dist_mm / 1000.0f;
            
            // Initialisation si première lecture
            if (drone->lidar_dist_m < 0) drone->lidar_dist_m = new_dist; 
            else drone->lidar_dist_m = (drone->lidar_dist_m * 0.70f) + (new_dist * 0.30f); // LPF
        } else {
            // Code erreur ou hors de portée
            drone->lidar_dist_m = -1.0f; 
        }
    }
    
    // === OPTICAL FLOW ===
    else if (msp_cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        // Payload: [Quality(1)][MotionX(4)][MotionY(4)]
        
        // 1. Gestion du Delta Temps (dt)
        if (prev_flow_time == 0) { 
            prev_flow_time = now; 
            return; 
        }
        
        float dt = (now - prev_flow_time) / 1000000.0f;
        prev_flow_time = now;
        
        // Sécurité: Si on a perdu des paquets et que dt est énorme, on ignore ce sample
        // pour éviter une injection de vitesse délirante.
        if (dt < 0.001f || dt > 0.5f) return; 

        uint8_t quality = msp_buffer[0];
        int32_t raw_counts_x = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        int32_t raw_counts_y = msp_buffer[5] | (msp_buffer[6] << 8) | (msp_buffer[7] << 16) | (msp_buffer[8] << 24);

        // 2. Conversion en Radian/s (Rate)
        // Velocity = Counts / (Scaler * dt)
        float velocity_x = (float)raw_counts_x / (OPTICAL_FLOW_SCALER * dt);
        float velocity_y = (float)raw_counts_y / (OPTICAL_FLOW_SCALER * dt);

        // 3. Filtrage (LPF)
        // Le capteur optique est bruité, le filtrage est crucial
        drone->flow_x_rad = (drone->flow_x_rad * 0.60f) + (velocity_x * 0.40f);
        drone->flow_y_rad = (drone->flow_y_rad * 0.60f) + (velocity_y * 0.40f);

        // 4. Deadband / Zero Snap
        if (fabsf(drone->flow_x_rad) < 0.01f) drone->flow_x_rad = 0.0f;
        if (fabsf(drone->flow_y_rad) < 0.01f) drone->flow_y_rad = 0.0f;
        
        drone->flow_quality = quality;
        drone->flow_valid = true;
    }
}

// --- MACHINE A ETATS MSP V2 NATIVE (Inspirée ArduPilot) ---
void parse_msp_byte(DroneState* drone, uint8_t c) {
    switch (msp_state) {
        // Attente du '$'
        case MSP_IDLE:
            if (c == MSP_HEADER_START) msp_state = MSP_HEADER_START_RECEIVED;
            break;

        // Attente du 'X' (MSPv2 Native)
        case MSP_HEADER_START_RECEIVED:
            if (c == MSP_V2_FRAME_ID) msp_state = MSP_HEADER_X_RECEIVED;
            else msp_state = MSP_IDLE;
            break;

        // Attente du '<' (Direction FC <--- Sensor)
        // C'est l'étape qui manquait dans votre ancien code !
        case MSP_HEADER_X_RECEIVED:
            if (c == MSP_DIRECTION_RX) {
                msp_idx = 0;
                msp_crc = 0; // Reset CRC
                msp_state = MSP_HEADER_V2_NATIVE;
            } else {
                msp_state = MSP_IDLE;
            }
            break;

        // Lecture Entête V2 (5 octets : Flag + CmdID(2) + Size(2))
        case MSP_HEADER_V2_NATIVE:
            msp_buffer[msp_idx++] = c;
            msp_crc = crc8_dvb_s2(msp_crc, c); // Le header fait partie du CRC

            if (msp_idx == 5) {
                // Décodage de l'entête
                // buffer[0] = flag (ignoré ici)
                msp_cmd_id = msp_buffer[1] | (msp_buffer[2] << 8);
                msp_payload_size = msp_buffer[3] | (msp_buffer[4] << 8);

                // Sécurité taille buffer
                if (msp_payload_size > sizeof(msp_buffer)) {
                    msp_state = MSP_IDLE; // Paquet trop gros
                } else {
                    msp_idx = 0;
                    // Si payload vide, on saute direct au checksum
                    msp_state = (msp_payload_size > 0) ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
                }
            }
            break;

        // Lecture Payload
        case MSP_PAYLOAD_V2_NATIVE:
            msp_buffer[msp_idx++] = c;
            msp_crc = crc8_dvb_s2(msp_crc, c); // Le payload fait partie du CRC
            
            if (msp_idx == msp_payload_size) {
                msp_state = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        // Vérification CRC
        case MSP_CHECKSUM_V2_NATIVE:
            if (msp_crc == c) {
                process_msp_packet(drone);
            }
            msp_state = MSP_IDLE;
            break;
            
        default:
            msp_state = MSP_IDLE;
            break;
    }
}

// --- TÂCHE FREERTOS ---
void FlowTaskCode(void * parameter) {
    DroneState* drone = (DroneState*)parameter;

    for(;;) { 
        // On lit tout ce qui arrive sur le port série
        // vTaskDelay gère le rythme, pas besoin de timeout complexe ici
        while (FlowSerial.available()) {
            uint8_t c = FlowSerial.read();
            parse_msp_byte(drone, c);
        }
        
        // Pause de 10ms (100Hz max de polling, suffisant pour du Flow à 30-50Hz)
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// --- INIT ---
void flow_init(DroneState* drone) {
    // Initialisation du pointeur LIDAR
    drone->lidar_dist_m = -1.0f;

    FlowSerial.setRxBufferSize(1024); 
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    
    xTaskCreatePinnedToCore(
        FlowTaskCode,   
        "FlowTask",     
        4096,          
        drone,          
        1,              
        &FlowTaskHandle,
        0 // Core 0 pour ne pas gêner le PID sur Core 1
    );
    
    Serial.println("FLOW: Init OK (MSP V2 Native State Machine)");
}

void flow_update(DroneState* drone) {
    // Vide, géré par FreeRTOS
}