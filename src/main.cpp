#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "alt_imu.h"
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
    Wire.setTimeOut(1);

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BATTERY, INPUT);
    analogReadResolution(12);

    motors_init();
    radio_init();
    radio_start_task(); // <-- AJOUT: radio indépendante de la loop()

    // Pour l'initialisation, on envoie 2000 aux ESC (Procédure standard)
    
    motors_write_direct(2000, 2000, 2000, 2000);


    // 3. DEMARRAGE TÂCHE TELEMETRIE (WIFI)
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
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        motors_stop();
        drone.current_mode = MODE_SAFE;

        imu_init();
        imu_start_task();      // <-- AJOUT: IMU sur tâche FreeRTOS

        alt_imu_init();
        alt_imu_calibrate_mag();  // Calibration magnétomètre (10s de rotation)
        alt_imu_start_task();  // <-- Alt IMU sur tâche FreeRTOS séparée

        pid_init();
        pid_init_params(&drone);
    }

    // Initialisation des compteurs de diag
    drone.max_time_radio = 0;
    drone.max_time_imu = 0;
    drone.max_time_pid = 0;
    
    loop_timer = micros();
}

void loop() {
    unsigned long t_start = micros();

    // Lecture tension batterie (filtrée)
    static float vbat_filter = 11.1f;
    int raw = analogRead(PIN_BATTERY);
    float v_pin = (raw / 4095.0f) * 3.3f;
    float v_bat = v_pin * BAT_SCALE;
    vbat_filter = (vbat_filter * 0.99f) + (v_bat * 0.01f);
    drone.voltage_bat = vbat_filter;

    radio_update(&drone);
    unsigned long t_radio = micros();

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
    } else {
        // Au lieu de bloquer sur I2C ici:
        // imu_read(&drone);

        unsigned long t_imu_start = micros();
        imu_update(&drone);      // <-- snapshot non-bloquant
        alt_imu_update(&drone);  // <-- snapshot alt_imu non-bloquant
        unsigned long t_imu = micros();
        (void)t_imu_start;   // durée "loop" IMU n'a plus de sens; drone.current_time_imu vient de la task

        switch(drone.current_mode) {
            // ---------------- MODE SAFE ----------------
            case MODE_SAFE:
                motors_stop();
                
                // ARMEMENTS : Gaz Bas + Yaw Gauche
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0; // Reset erreurs
                        pid_reset_integral();
                        drone.angle_pitch = 0; 
                        drone.angle_roll = 0;
                        imu_request_reset();      // <-- AJOUT (très important)
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

                // --- INTELLIGENCE DE VOL ---
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // --- DESARMEMENT MANUEL D'URGENCE ---
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

                // --- SECURITE RADIO / AUTOLANDING ---
                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    
                    if (millis() - disarm_debounce_timer > 60000) { 
                        drone.current_mode = MODE_ARMED; 
                        error_code = 2; // PERTE RADIO
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
                // Sortie du mode test si on touche aux gaz
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }

        // --- DIAGNOSTIC LAG (BOÎTE NOIRE) ---
        unsigned long t_end = micros();
        unsigned long dur_radio = t_radio - t_start;
        unsigned long dur_imu = t_imu - t_radio;
        unsigned long dur_pid_mix = t_end - t_imu;
        unsigned long total_loop = t_end - t_start;

        drone.loop_time = total_loop; 
        
        // Si on détecte un gros lag (> 6000us), on enregistre le coupable
        // MAIS on évite d'overwrite le code d'erreur 888888 (IMU CRASH)
        if (total_loop > 6000) {
            if(dur_radio > drone.max_time_radio) drone.max_time_radio = dur_radio;
            if(drone.max_time_imu != 888888 && dur_imu > drone.max_time_imu) drone.max_time_imu = dur_imu;
            if(dur_pid_mix > drone.max_time_pid) drone.max_time_pid = dur_pid_mix;
        }
    }

    // Gestion Loop Time constant
    unsigned long time_used = micros() - loop_timer;
    
    // Si on est en avance, on attend
    if (time_used < LOOP_TIME_US) {
        if (LOOP_TIME_US - time_used > 2000) delay(1);
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    loop_timer = micros();
}