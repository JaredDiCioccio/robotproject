#ifndef IMU_H
#define IMU_H

#include <rc/mpu.h>

#include "robot_app.h"
extern rc_mpu_data_t imuData;

void *imu_updater(void *unused);
void robot_imu_init(RobotState *robotState);

#endif