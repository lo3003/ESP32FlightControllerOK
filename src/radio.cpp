#include <Arduino.h>
#include "radio.h"
#include "config.h"
#include "sbus.h" 

// Variables globales accessibles par setup_wizard
// Initialisation à 1500 (neutre) sauf gaz à 0
int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 0, raw_channel_4 = 1500;

bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

void radio_init() {
    sbus.Begin();
}

// Fonction magique qui "écrase" les bords pour garantir 1000 et 2000
// MODIFICATION SECURITE : Retourne -1 si l'entrée est invalide (glitch à 0)
int process_channel(int input_sbus, bool reverse) {
    // --- SECURITE ANTI-GLITCH ---
    // Si la valeur S.BUS est proche de 0 (ex: câble débranché ou glitch), on rejette !
    // Une valeur S.BUS valide est généralement > 172.
    if (input_sbus < 50) {
        return -1; // Code d'erreur
    }

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
        // Variables temporaires pour vérifier la validité AVANT d'appliquer
        int t_roll  = process_channel(data.ch[3], false); 
        int t_pitch = process_channel(data.ch[1], true); 
        int t_thr   = process_channel(data.ch[2], false); 
        int t_yaw   = process_channel(data.ch[0], false);

        // --- SECURITE INTEGRITE ---
        // On ne met à jour les globales QUE si TOUS les canaux sont valides (> -1)
        // Cela empêche un glitch unique de couper les gaz
        if (t_roll != -1 && t_pitch != -1 && t_thr != -1 && t_yaw != -1) {
            raw_channel_1 = t_roll;
            raw_channel_2 = t_pitch;
            raw_channel_3 = t_thr;
            raw_channel_4 = t_yaw;
        }
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