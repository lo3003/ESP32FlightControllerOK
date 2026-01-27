#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "alt_imu.h"
#include "yaw_fusion.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"
#include "telemetry.h"
#include "flow.h" 

// --- FLAG POUR DESACTIVER LA FUSION YAW ---
#define YAW_FUSION_ENABLED 1

DroneState drone;
unsigned long loop_timer;
unsigned long arming_timer = 0;
unsigned long disarm_debounce_timer = 0; 
unsigned long angle_security_timer = 0;  
int error_code = 0;                      

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    Wire.setTimeOut(1);

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BATTERY, INPUT);
    analogReadResolution(12);

    // Initialisation Pins Moteurs (Sécurité)
    pinMode(12, OUTPUT); digitalWrite(12, LOW);
    pinMode(13, OUTPUT); digitalWrite(13, LOW);
    pinMode(14, OUTPUT); digitalWrite(14, LOW);
    pinMode(15, OUTPUT); digitalWrite(15, LOW);

    radio_init();
    radio_start_task(); 
    
    // --- 1. LANCEMENT TACHE DE FOND FLOW (FreeRTOS) ---
    // La lecture se fera désormais toute seule sur le Coeur 0
    flow_init(&drone); 

    // --- 2. LANCEMENT TACHE DE FOND WIFI ---
    start_telemetry_task(&drone); 

    // --- 3. ATTENTE SIGNAL RADIO (Le "Stress" rapide) ---
    // Si la radio n'est pas allumée, ça clignote très vite ici. C'est NORMAL.
    unsigned long wait_start = millis();
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        if((millis() / 50) % 2) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        delay(5);
        if(millis() - wait_start > 15000) break; 
    }

    // 4. DECISION SELON LE STICK
    if(drone.channel_3 > 1900) {
        // MODE CALIBRATION ESC
        motors_init(); 
        motors_write_direct(2000, 2000, 2000, 2000);
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        // MODE VOL NORMAL
        drone.current_mode = MODE_SAFE;

        // Calibration MPU6050
        imu_init();            
        
        // Calibration AltIMU
        alt_imu_init();        

        // Pause visuelle (LED Eteinte)
        digitalWrite(PIN_LED, LOW);
        delay(1000);

        // Calibration Magnétomètre (LED Clignote lentement ou s'éteint)
        alt_imu_calibrate_mag();  

        // Fin de calibration - LED fixe 1 seconde
        digitalWrite(PIN_LED, HIGH);
        delay(1000);
        digitalWrite(PIN_LED, LOW);

        motors_init(); 
        
        imu_start_task();      
        alt_imu_start_task();  
        yaw_fusion_init();

        pid_init();
        pid_init_params(&drone);
    }

    drone.max_time_radio = 0;
    drone.max_time_imu = 0;
    drone.max_time_pid = 0;
    
    loop_timer = micros();
}

void loop() {
    unsigned long t_start = micros();

    // Lecture tension batterie (10Hz)
    static uint8_t bat_counter = 0;
    static float vbat_filter = 11.1f;
    if (++bat_counter >= 25) {
        bat_counter = 0;
        int raw = analogRead(PIN_BATTERY);
        float v_pin = (raw / 4095.0f) * 3.3f;
        float v_bat = v_pin * BAT_SCALE;
        vbat_filter = (vbat_filter * 0.95f) + (v_bat * 0.05f);
        drone.voltage_bat = vbat_filter;
    }

    radio_update(&drone);
    
    // --- CORRECTION MAJEURE ---
    // flow_update(&drone);  <-- SUPPRIMÉ ! 
    // On ne l'appelle PLUS ici car la tâche FreeRTOS le fait déjà en arrière-plan.
    // Le laisser créait le conflit et le lag.
    
    unsigned long t_radio = micros();

    // Gestion LED (Heartbeat normal 500ms)
    if (error_code > 0) {
        if ((millis() % 50) < 25) digitalWrite(PIN_LED, HIGH);
        else digitalWrite(PIN_LED, LOW);
    } 
    else if (drone.current_mode == MODE_SAFE) {
        // Clignotement lent en SAFE pour dire "Je suis vivant mais désarmé"
        if ((millis() % 1000) < 500) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
    }

    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } else {
        unsigned long t_imu_start = micros();
        imu_update(&drone);      
        alt_imu_update(&drone);  

#if YAW_FUSION_ENABLED
        static uint8_t fusion_counter = 0;
        if (drone.current_mode == MODE_ARMED || drone.current_mode == MODE_FLYING) {
            if (++fusion_counter >= 5) {
                fusion_counter = 0;
                const float dt_s = LOOP_TIME_US * 5.0f * 1e-6f;
                yaw_fusion_update(&drone, dt_s);
            }
        } else {
            fusion_counter = 0;
        }
#endif

        unsigned long t_imu = micros();
        (void)t_imu_start;

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                // ARMEMENTS : Gaz Bas + Yaw Gauche
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0;
                        pid_reset_integral();
                        drone.angle_pitch = 0;
                        drone.angle_roll = 0;
                        imu_request_reset();      
#if YAW_FUSION_ENABLED
                        yaw_fusion_reset(&drone); 
#endif
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            case MODE_ARMED:
                motors_stop(); 
                // Clignotement d'avertissement plus rapide
                if ((millis() % 200) < 100) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);

                if(drone.channel_3 > 1040) {
                    drone.current_mode = MODE_FLYING;
                    disarm_debounce_timer = 0;
                    angle_security_timer = 0;
                }
                
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        arming_timer = 0;
                     }
                } else { arming_timer = 0; }
                break;

            case MODE_FLYING:
                digitalWrite(PIN_LED, LOW); // Eteint en vol pour économiser/ne pas éblouir

                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        motors_stop(); 
                        arming_timer = 0;
                     }
                } else { 
                    arming_timer = 0; 
                }

                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    if (millis() - disarm_debounce_timer > 60000) { 
                        drone.current_mode = MODE_ARMED; 
                        error_code = 2; 
                        disarm_debounce_timer = 0;
                    }
                } else {
                    disarm_debounce_timer = 0; 
                }
                break;

            case MODE_WEB_TEST:
                motors_write_direct(
                    drone.web_test_vals[1], drone.web_test_vals[2], 
                    drone.web_test_vals[3], drone.web_test_vals[4]
                );
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }

        // --- DIAGNOSTIC LAG ---
        unsigned long t_end = micros();
        unsigned long dur_radio = t_radio - t_start;
        unsigned long dur_imu = t_imu - t_radio;
        unsigned long dur_pid_mix = t_end - t_imu;
        unsigned long total_loop = t_end - t_start;

        drone.loop_time = total_loop; 
        
        if (total_loop > 6000) {
            if(dur_radio > drone.max_time_radio) drone.max_time_radio = dur_radio;
            if(drone.max_time_imu != 888888 && dur_imu > drone.max_time_imu) drone.max_time_imu = dur_imu;
            if(dur_pid_mix > drone.max_time_pid) drone.max_time_pid = dur_pid_mix;
        }
    }

    // --- YIELD WIFI / FREERTOS ---
    unsigned long time_now = micros();
    if (loop_timer + LOOP_TIME_US > time_now) {
        unsigned long remaining = (loop_timer + LOOP_TIME_US) - time_now;
        // On rend la main plus généreusement pour le WiFi et le Flow
        if (remaining > 1000) {
             vTaskDelay(1); 
        }
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    
    loop_timer = micros();
}