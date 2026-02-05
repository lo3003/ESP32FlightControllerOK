#ifndef FLOW_H
#define FLOW_H

#include "types.h"

/**
 * @brief Initialise le driver MSP v2 pour Matek 3901-L0X
 *        Lance une tâche FreeRTOS sur Core 0 pour la lecture UART
 * @param drone Pointeur vers l'état global du drone
 */
void flow_init(DroneState* drone);

/**
 * @brief Copie les données MSP décodées dans DroneState
 *        Appelé depuis la boucle principale (non-bloquant)
 * @param drone Pointeur vers l'état global du drone
 */
void flow_update(DroneState* drone);

/**
 * @brief Calcule la vitesse linéaire estimée par fusion Flow + Gyro + Lidar
 *        Formule: V = (flow_rad/s - gyro_rad/s) * altitude_m
 *        Doit être appelé juste avant pid_compute() en MODE_FLYING
 * @param drone Pointeur vers l'état global du drone
 */
void flow_compute_velocity(DroneState* drone);

#endif
