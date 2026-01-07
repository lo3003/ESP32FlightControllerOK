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

    start_telemetry_task(&drone); // Optionnel

    Serial.println(F("Attente Radio... Moteurs Maintenus à MAX (2000us)"));
    
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
        Serial.println(F(">>> MODE CALIBRATION CONFIRME <<<"));
        // On reste à 2000us
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        // Le stick est en bas (ou l'utilisateur l'a baissé)
        // -> On envoie 1000us.
        // Si c'était un démarrage normal, le drone s'arme silencieusement.
        // Si c'était une calibration, cela valide le point bas (Long Bip).
        Serial.println(F("Démarrage Normal / Fin Calibration"));
        motors_stop(); 
        
        drone.current_mode = MODE_SAFE;
        imu_init(); 
        pid_init();
    }
    
    loop_timer = micros();
}

void loop() {
    radio_update(&drone);

    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } 
    else {
        imu_read(&drone);

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                digitalWrite(PIN_LED, HIGH);
                if(drone.channel_3 < 1200 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        pid_reset_integral();
                        drone.angle_pitch = 0; drone.angle_roll = 0;
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            case MODE_ARMED:
                motors_stop(); // <--- CHANGEMENT : On force l'arrêt au lieu du ralenti
                digitalWrite(PIN_LED, LOW);
                
                // CHANGEMENT : On décolle plus tôt (dès 1040 au lieu de 1200)
                // Cela permet une transition plus douce sans zone morte
                if(drone.channel_3 > 1040) drone.current_mode = MODE_FLYING;
                
                // Désarmement (Code existant)
                if(drone.channel_3 < 1050 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        arming_timer = 0;
                     }
                } else { arming_timer = 0; }
                break;

            case MODE_FLYING:
                // On réactive l'intelligence de vol
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // Sécurité désarmement en vol
                if(drone.channel_3 < 1100) drone.current_mode = MODE_ARMED;
                break;

            // ... après le case MODE_FLYING
            case MODE_WEB_TEST:
                // On applique directement les valeurs reçues du Wifi
                motors_write_direct(
                    drone.web_test_vals[1], 
                    drone.web_test_vals[2], 
                    drone.web_test_vals[3], 
                    drone.web_test_vals[4]
                );
                
                // SÉCURITÉ : Si on touche au stick des gaz, on coupe tout immédiatement !
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }
    }

    unsigned long time_used = micros() - loop_timer;
    if (time_used < LOOP_TIME_US) {
        if (LOOP_TIME_US - time_used > 2000) delay(1);
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    loop_timer = micros();
}