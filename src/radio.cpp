#include <Arduino.h>
#include "radio.h"
#include "config.h"
#include "sbus.h" 

// Variables globales accessibles par setup_wizard
int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 0, raw_channel_4 = 1500;

bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

void radio_init() {
    sbus.Begin();
}

// Fonction magique qui "écrase" les bords pour garantir 1000 et 2000
int process_channel(int input_sbus, bool reverse) {
    // VOS VALEURS REELLES : env. 310 à 1690
    // ON TRICHE : On dit que le min est 360 et le max 1640 pour être sûr d'atteindre les bouts.
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
    // --- CORRECTIF LAG & SECURITE ---
    // Au lieu de lire un seul paquet (if), on boucle (while) pour vider le buffer.
    // Si la boucle loop() a pris du retard, plusieurs paquets S.BUS se sont empilés.
    // On les lit tous jusqu'au dernier pour avoir l'ordre le plus récent.
    
    bool new_data = false;
    while (sbus.Read()) {
        // --- NOUVEAU : FILTRAGE FAILSAFE ---
        // Si le paquet indique une perte de frame ou un failsafe, on l'ignore !
        if (sbus.data().failsafe || sbus.data().lost_frame) {
            continue; 
        }

        data = sbus.data();
        new_data = true; // On a reçu au moins un paquet frais ET VALIDE
    }

    if (new_data) {
        // On met à jour les variables globales UNIQUEMENT si on a reçu des données fraîches
        
        // ROLL (Ch 3)
        raw_channel_1 = process_channel(data.ch[3], false); 
        
        // PITCH (Ch 1) 
        raw_channel_2 = process_channel(data.ch[1], true); 
        
        // THROTTLE (Ch 2)
        raw_channel_3 = process_channel(data.ch[2], false); 
        
        // YAW (Ch 0)
        raw_channel_4 = process_channel(data.ch[0], false);
    }
}

void radio_update(DroneState *drone) {
    // Lit la radio (et vide le buffer grâce au correctif ci-dessus)
    radio_read_raw();

    // Copie vers l'état du drone
    if (raw_channel_3 == 0) {
        drone->channel_3 = 0; // Sécurité si jamais initialisé à 0
    } else {
        drone->channel_1 = raw_channel_1;
        drone->channel_2 = raw_channel_2;
        drone->channel_3 = raw_channel_3;
        drone->channel_4 = raw_channel_4;
    }
}

int convert_receiver_channel(byte function) { return 1500; }