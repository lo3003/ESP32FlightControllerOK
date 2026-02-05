#include <Arduino.h>
#include "flow.h"
#include "config.h"
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// --- CONFIGURATION ---
static HardwareSerial FlowSerial(1); // UART1

// --- CONSTANTES MSP V2 ---
#define MSP_HEADER_START '$'
#define MSP_V2_FRAME_ID  'X'
#define MSP_DIRECTION_RX '<'
#define MSP2_SENSOR_RANGEFINDER 0x1F01 
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02 

// --- STRUCTURE DE SYNCHRONISATION (SNAPSHOT) ---
typedef struct {
    float flow_x_rad;
    float flow_y_rad;
    uint8_t flow_quality;
    float lidar_dist_m;
    bool flow_valid;
    unsigned long last_update_ms;
} FlowSnapshot;

static FlowSnapshot flow_snap = {0.0f, 0.0f, 0, -1.0f, false, 0};
static portMUX_TYPE flow_mux = portMUX_INITIALIZER_UNLOCKED;

// Variables de travail locales (Core 0)
static float local_flow_x = 0.0f;
static float local_flow_y = 0.0f;
static float local_lidar = -1.0f;
static unsigned long prev_flow_time = 0;

// On utilise un scaler très faible pour garder de la précision
// Si c'est encore trop faible, on pourra descendre à 1.0
#define OPTICAL_FLOW_SENSITIVITY 10.0f 

enum mspState_e { MSP_IDLE, MSP_HEADER_START_RECEIVED, MSP_HEADER_X_RECEIVED, MSP_HEADER_V2_NATIVE, MSP_PAYLOAD_V2_NATIVE, MSP_CHECKSUM_V2_NATIVE };
static mspState_e msp_state = MSP_IDLE;
static uint8_t msp_crc = 0;
static uint16_t msp_idx = 0;
static uint16_t msp_payload_size = 0;
static uint16_t msp_cmd_id = 0;
static uint8_t msp_buffer[128];

TaskHandle_t FlowTaskHandle = NULL;

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

void process_msp_packet(DroneState* drone) {
    unsigned long now = micros();

    if (msp_cmd_id == MSP2_SENSOR_RANGEFINDER) {
        int32_t dist_mm = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        if (dist_mm >= 0 && dist_mm < 5000) {
            float new_dist = dist_mm / 1000.0f;
            if (local_lidar < 0) local_lidar = new_dist;
            else local_lidar = (local_lidar * 0.70f) + (new_dist * 0.30f);
        } else {
            local_lidar = -1.0f;
        }

        portENTER_CRITICAL(&flow_mux);
        flow_snap.lidar_dist_m = local_lidar;
        flow_snap.last_update_ms = millis();
        portEXIT_CRITICAL(&flow_mux);
    }
    else if (msp_cmd_id == MSP2_SENSOR_OPTIC_FLOW) {
        uint8_t quality = msp_buffer[0];
        
        // Lecture des counts bruts (accumulation de mouvement depuis le dernier message)
        int32_t raw_x = msp_buffer[1] | (msp_buffer[2] << 8) | (msp_buffer[3] << 16) | (msp_buffer[4] << 24);
        int32_t raw_y = msp_buffer[5] | (msp_buffer[6] << 8) | (msp_buffer[7] << 16) | (msp_buffer[8] << 24);

        // CHANGEMENT MAJEUR : On ne divise plus par dt.
        // On convertit les counts en une valeur "Rate" exploitable.
        float vel_x = (float)raw_x / OPTICAL_FLOW_SENSITIVITY;
        float vel_y = (float)raw_y / OPTICAL_FLOW_SENSITIVITY;

        // Filtrage LPF à 50% pour la réactivité
        local_flow_x = (local_flow_x * 0.50f) + (vel_x * 0.50f);
        local_flow_y = (local_flow_y * 0.50f) + (vel_y * 0.50f);

        // Deadband très fine (on veut détecter le moindre glissement)
        if (fabsf(local_flow_x) < 0.005f) local_flow_x = 0.0f;
        if (fabsf(local_flow_y) < 0.005f) local_flow_y = 0.0f;

        portENTER_CRITICAL(&flow_mux);
        flow_snap.flow_x_rad = local_flow_x;
        flow_snap.flow_y_rad = local_flow_y;
        flow_snap.flow_quality = quality;
        flow_snap.flow_valid = (quality > 50);
        flow_snap.last_update_ms = millis();
        portEXIT_CRITICAL(&flow_mux);
    }
}

void parse_msp_byte(DroneState* drone, uint8_t c) {
    switch (msp_state) {
        case MSP_IDLE: if (c == MSP_HEADER_START) msp_state = MSP_HEADER_START_RECEIVED; break;
        case MSP_HEADER_START_RECEIVED: if (c == MSP_V2_FRAME_ID) msp_state = MSP_HEADER_X_RECEIVED; else msp_state = MSP_IDLE; break;
        case MSP_HEADER_X_RECEIVED: if (c == MSP_DIRECTION_RX) { msp_idx = 0; msp_crc = 0; msp_state = MSP_HEADER_V2_NATIVE; } else msp_state = MSP_IDLE; break;
        case MSP_HEADER_V2_NATIVE:
            msp_buffer[msp_idx++] = c; msp_crc = crc8_dvb_s2(msp_crc, c);
            if (msp_idx == 5) {
                msp_cmd_id = msp_buffer[1] | (msp_buffer[2] << 8);
                msp_payload_size = msp_buffer[3] | (msp_buffer[4] << 8);
                if (msp_payload_size > sizeof(msp_buffer)) msp_state = MSP_IDLE;
                else { msp_idx = 0; msp_state = (msp_payload_size > 0) ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE; }
            }
            break;
        case MSP_PAYLOAD_V2_NATIVE: msp_buffer[msp_idx++] = c; msp_crc = crc8_dvb_s2(msp_crc, c); if (msp_idx == msp_payload_size) msp_state = MSP_CHECKSUM_V2_NATIVE; break;
        case MSP_CHECKSUM_V2_NATIVE: if (msp_crc == c) process_msp_packet(drone); msp_state = MSP_IDLE; break;
        default: msp_state = MSP_IDLE; break;
    }
}

void FlowTaskCode(void * parameter) {
    DroneState* drone = (DroneState*)parameter;
    for(;;) { 
        int bytes_read = 0;
        while (FlowSerial.available() && bytes_read < 32) {
            parse_msp_byte(drone, FlowSerial.read());
            bytes_read++;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS); 
    }
}

void flow_init(DroneState* drone) {
    drone->lidar_dist_m = -1.0f;
    FlowSerial.setRxBufferSize(1024); 
    FlowSerial.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    
    xTaskCreatePinnedToCore(FlowTaskCode, "FlowTask", 4096, drone, 2, &FlowTaskHandle, 0);
    Serial.println("FLOW: Sensitive Mode OK");
}

void flow_update(DroneState* drone) {
    FlowSnapshot s;
    portENTER_CRITICAL(&flow_mux);
    s = flow_snap;
    portEXIT_CRITICAL(&flow_mux);

    if (millis() - s.last_update_ms > 200) {
        drone->flow_valid = false;
        drone->flow_quality = 0;
    } else {
        drone->flow_x_rad = s.flow_x_rad;
        drone->flow_y_rad = s.flow_y_rad;
        drone->flow_quality = s.flow_quality;
        drone->lidar_dist_m = s.lidar_dist_m;
        drone->flow_valid = s.flow_valid;
    }
}