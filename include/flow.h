#ifndef FLOW_H
#define FLOW_H

#include "types.h"

// On passe le pointeur drone à l'init pour la tâche FreeRTOS
void flow_init(DroneState* drone); 

// On garde cette fonction mais elle ne fera plus rien de lourd
void flow_update(DroneState* drone);

#endif