#include <Arduino.h>
#include "radio.h"
#include "config.h"
#include "sbus.h" 

// Initialisation à 0 pour la sécurité démarrage
int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 0, raw_channel_4 = 1500;

bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

void radio_init() {
    sbus.Begin();
}

// Fonction magique qui "écrase" les bords pour garantir 1000 et 2000
int process_channel(int input_sbus, bool reverse) {
    // VOS VALEURS REELLES : env. 310 à 1690
    // ON TRICHE : On dit que le min est 360 et le max 1640.
    int min_safe = 360; 
    int max_safe = 1640;
    
    // Tout ce qui est en dessous de 360 deviendra 1000 pile.
    // Tout ce qui est au dessus de 1640 deviendra 2000 pile.
    int val = map(input_sbus, min_safe, max_safe, 1000, 2000);
    
    if(reverse) val = 3000 - val; // Inversion (2000+1000 - val)

    // Sécurité bornes ultimes
    return constrain(val, 1000, 2000);
}

void radio_read_raw() {
    if (sbus.Read()) {
        data = sbus.data();
        
        // ROLL (Ch 3)
        raw_channel_1 = process_channel(data.ch[3], false); 
        
        // PITCH (Ch 1) - Inversé pour que Stick Haut = Piquer (Valeur basse)
        // Vérifiez sur le Dashboard : Stick HAUT doit diminuer la valeur.
        raw_channel_2 = process_channel(data.ch[1], true); 
        
        // THROTTLE (Ch 2)
        raw_channel_3 = process_channel(data.ch[2], false); 
        
        // YAW (Ch 0)
        raw_channel_4 = process_channel(data.ch[0], false);
    }
}

void radio_update(DroneState *drone) {
    radio_read_raw();

    if (raw_channel_3 == 0) {
        drone->channel_3 = 0;
    } else {
        drone->channel_1 = raw_channel_1;
        drone->channel_2 = raw_channel_2;
        drone->channel_3 = raw_channel_3;
        drone->channel_4 = raw_channel_4;
    }
}

int convert_receiver_channel(byte function) { return 1500; }