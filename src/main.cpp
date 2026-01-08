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
    
    // 1. INIT MOTEURS A 2000us (MAX)
    // Les ESCs croient que vous avez le stick en haut (Procédure PDF)
    motors_init(); 
    
    // 2. Init Radio
    radio_init();

    motors_write_direct(2000, 2000, 2000, 2000);

    //start_telemetry_task(&drone); // Optionnel

    
    
    // On attend un signal valide. Pendant ce temps, les ESC sont en attente "Max Throttle"
    // Ils vont faire "123" puis attendre les Bips de confirmation.
    unsigned long wait_start = millis();
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        // Clignotement rapide
        if((millis() / 50) % 2) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        delay(5);
        if(millis() - wait_start > 15000) break; // Sécurité 15s
    }

    // 3. DECISION SELON LE STICK
    if(drone.channel_3 > 1900) {
        // Le stick est vraiment en haut -> On continue la Calibration
        // On reste à 2000us
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        // Le stick est en bas (ou l'utilisateur l'a baissé)
        // -> On envoie 1000us.
        // Si c'était un démarrage normal, le drone s'arme silencieusement.
        // Si c'était une calibration, cela valide le point bas (Long Bip).
        motors_stop(); 
        
        drone.current_mode = MODE_SAFE;
        imu_init(); 
        pid_init();
        pid_init_params(&drone);
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Mise à jour Radio (Lit les sticks)
    radio_update(&drone);

    // 2. Gestion Globale LED (Code Erreur visuel)
    // Si error_code > 0, on a eu un crash/failsafe -> Clignotement STROBO RAPIDE
    if (error_code > 0) {
        // Clignote très vite (20 fois par seconde) pour dire "ERREUR CRITIQUE"
        if ((millis() % 50) < 25) digitalWrite(PIN_LED, HIGH);
        else digitalWrite(PIN_LED, LOW);
    } 
    // Sinon, comportement normal selon le mode
    else if (drone.current_mode == MODE_SAFE) {
        digitalWrite(PIN_LED, HIGH); // Fixe en SAFE
    }
    // (En vol, la LED sera gérée ailleurs ou éteinte)


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
                
                // Réarmement (Stick Gaz en bas + Yaw à gauche par ex, ou juste Gaz < 1010)
                // Ici on garde votre logique simple : Attente stick bas
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0; // ON EFFACE L'ERREUR AU REARMEMENT !
                        pid_reset_integral();
                        drone.angle_pitch = 0; drone.angle_roll = 0;
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE ARMED ----------------
            case MODE_ARMED:
                motors_stop(); 
                // Clignotement lent pour dire "PRET"
                if ((millis() % 500) < 100) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);

                // Décollage
                if(drone.channel_3 > 1040) {
                    drone.current_mode = MODE_FLYING;
                    // Reset des timers de sécurité au décollage
                    disarm_debounce_timer = 0;
                    angle_security_timer = 0;
                }
                
                // Désarmement (Manche gauche coin bas-gauche ou juste bas)
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
                // LED Eteinte en vol (ou heartbeat discret)
                digitalWrite(PIN_LED, LOW); 

                // --- 1. SECURITE CRASH (TEMPORISÉE) ---
                // Si l'angle dépasse 70°, on lance un chrono.
                // Si ça reste > 70° pendant 300ms, ALORS on coupe.
                if (abs(drone.angle_roll) > 70 || abs(drone.angle_pitch) > 70) {
                    if(angle_security_timer == 0) angle_security_timer = millis();
                    
                    if(millis() - angle_security_timer > 1000) { // Délai de tolérance (1000ms)
                        motors_stop();
                        drone.current_mode = MODE_SAFE;
                        error_code = 1; // 1 = CRASH ANGLE
                        Serial.println("URGENCE: Angle > 70 deg confirmé (300ms)");
                        break; 
                    }
                } else {
                    // L'angle est revenu normal, on reset le chrono
                    angle_security_timer = 0;
                }

                // --- 2. INTELLIGENCE DE VOL ---
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // --- 3. SECURITE RADIO (TEMPORISÉE) ---
                // Si Gaz < 1010 pendant plus de 500ms -> Failsafe
                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    
                    if (millis() - disarm_debounce_timer > 1000) { // Délai tolérance (500ms)
                        drone.current_mode = MODE_ARMED; // Ou SAFE si vous préférez
                        error_code = 2; // 2 = PERTE RADIO
                        disarm_debounce_timer = 0;
                    }
                } else {
                    disarm_debounce_timer = 0; 
                }
                break;

            case MODE_WEB_TEST:
                // ... (Votre code Web Test inchangé) ...
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
    }

    // Gestion Loop Time (inchangée)
    unsigned long time_used = micros() - loop_timer;
    if (time_used < LOOP_TIME_US) {
        if (LOOP_TIME_US - time_used > 2000) delay(1);
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    loop_timer = micros();
}