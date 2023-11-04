#ifndef APP_H
#define APP_H

#include "imu.h"

typedef struct RobotStatus
{
    ImuStatus *imuStatus;
    bool systemError;
} RobotStatus;



extern uint8_t running;

extern pthread_mutex_t robotStatusMutex;
extern RobotStatus robotStatus;

extern pthread_mutex_t imuDataMutex;
extern rc_mpu_data_t imuData;

#endif