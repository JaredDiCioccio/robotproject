#ifndef APP_H
#define APP_H

#include "robot_imu.h"
#include "robot_app.h"
#include "robot_battery.h"

typedef struct RobotStatus
{
    ImuStatus *imuStatus;
    BatteryStatus *batteryStatus;
    bool systemError;
} RobotStatus;

typedef struct RobotState
{
    rc_mpu_data_t *imuData;
} RobotState;

extern uint8_t running;

extern pthread_mutex_t robotStatusMutex;
extern RobotStatus robotStatus;

extern pthread_mutex_t imuDataMutex;
extern rc_mpu_data_t imuData;

#endif