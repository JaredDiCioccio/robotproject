#ifndef IMU_H
#define IMU_H

#include <rc/mpu.h>

extern rc_mpu_data_t imuData;

typedef struct ImuStatus
{
    bool accelError;
    bool initError;
    bool gyroError;
} ImuStatus;

void *imu_updater(void *unused);

#endif