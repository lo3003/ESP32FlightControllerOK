#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"
#include "telemetry.h"

DroneState drone;
unsigned long loop_timer;
unsigned long arming_timer = 0;
unsigned long disarm_debounce_timer = 0; // Chrono pour la coupure Radio
unsigned long angle_security_timer = 0;  // Chrono pour l'angle excessif
int error_code = 0;                      // 0=OK, 1=CRASH ANGLE, 2=PERTE RADIO

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    
    pinMode(PIN_LED, OUTPUT);
    
    // 1. INIT MOTEURS
    motors_init(); 
    
    // 2. Init Radio
    radio_init();

    // Pour l'initialisation, on envoie 2000 aux ESC (Procédure standard)
    motors_write_direct(2000, 2000, 2000, 2000);

    // 3. DEMARRAGE TÂCHE TELEMETRIE (WIFI)
    // Indispensable pour voir le Loop Time sur le Web
    start_telemetry_task(&drone); 

    // On attend un signal valide. Pendant ce temps, les ESC sont en attente "Max Throttle"
    unsigned long wait_start = millis();
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        // Clignotement rapide
        if((millis() / 50) % 2) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        delay(5);
        if(millis() - wait_start > 15000) break; // Sécurité 15s
    }

    // 4. DECISION SELON LE STICK
    if(drone.channel_3 > 1900) {
        // Le stick est vraiment en haut -> On continue la Calibration ESC
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        // Le stick est en bas -> Démarrage normal
        motors_stop(); 
        
        drone.current_mode = MODE_SAFE;
        imu_init(); 
        pid_init();
        pid_init_params(&drone);
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Mise à jour Radio
    radio_update(&drone);

    // 2. Gestion LED Erreur
    if (error_code > 0) {
        if ((millis() % 50) < 25) digitalWrite(PIN_LED, HIGH);
        else digitalWrite(PIN_LED, LOW);
    } 
    else if (drone.current_mode == MODE_SAFE) {
        digitalWrite(PIN_LED, HIGH); // Fixe en SAFE
    }


    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } 
    else {
        // Lecture IMU
        imu_read(&drone);

        switch(drone.current_mode) {
            // ---------------- MODE SAFE ----------------
            case MODE_SAFE:
                motors_stop();
                
                // ARMEMENTS : Gaz Bas + Yaw Gauche (Stick gauche dans le coin bas-gauche)
                // Ou simplement Gaz < 1010 et Yaw < 1200
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0; // Reset erreurs
                        pid_reset_integral();
                        drone.angle_pitch = 0; drone.angle_roll = 0;
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE ARMED ----------------
            case MODE_ARMED:
                motors_stop(); 
                // Clignotement lent
                if ((millis() % 500) < 100) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);

                // Décollage (Gaz > 1040)
                if(drone.channel_3 > 1040) {
                    drone.current_mode = MODE_FLYING;
                    disarm_debounce_timer = 0;
                    angle_security_timer = 0;
                }
                
                // Désarmement (Gaz Bas + Yaw Droite)
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        arming_timer = 0;
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE FLYING ----------------
            case MODE_FLYING:
                digitalWrite(PIN_LED, LOW); 

                // --- 1. INTELLIGENCE DE VOL ---
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // --- 2. DESARMEMENT MANUEL D'URGENCE (AJOUT) ---
                // Si Gaz < 1010 ET Yaw à Droite (> 1800) pendant 1 seconde
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        motors_stop(); // Coupure immédiate
                        arming_timer = 0;
                     }
                } else { 
                    // Si on ne fait pas la combinaison, on reset le timer manuel
                    // On ne touche pas au timer disarm_debounce_timer ci-dessous
                    arming_timer = 0; 
                }

                // --- 3. SECURITE RADIO / AUTOLANDING ---
                // Si Gaz < 1010 (sans toucher au Yaw) pendant 60s
                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    
                    if (millis() - disarm_debounce_timer > 60000) { // 60 secondes pour test banc
                        drone.current_mode = MODE_ARMED; 
                        error_code = 2; // PERTE RADIO
                        disarm_debounce_timer = 0;
                    }
                } else {
                    disarm_debounce_timer = 0; 
                }
                
                /* // Bloc Crash Angle désactivé pour vos tests
                if (abs(drone.angle_roll) > 70 || abs(drone.angle_pitch) > 70) { ... }
                */
                break;

            case MODE_WEB_TEST:
                motors_write_direct(
                    drone.web_test_vals[1], drone.web_test_vals[2], 
                    drone.web_test_vals[3], drone.web_test_vals[4]
                );
                // Sortie du mode test si on touche aux gaz
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }
    }

    // Gestion Loop Time
    unsigned long time_used = micros() - loop_timer;
    
    // --- ENVOI VERS TELEMETRIE ---
    drone.loop_time = time_used; 
    // -----------------------------

    if (time_used < LOOP_TIME_US) {
        if (LOOP_TIME_US - time_used > 2000) delay(1);
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    loop_timer = micros();
}