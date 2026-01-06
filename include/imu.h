#ifndef IMU_H
#define IMU_H
#include "types.h"

void imu_init();
void imu_read(DroneState *drone);

#endif