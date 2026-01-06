#ifndef PID_H
#define PID_H
#include "types.h"

void pid_init();
void pid_reset_integral();
void pid_compute_setpoints(DroneState *drone);
void pid_compute(DroneState *drone);

#endif