#ifndef APP_H
#define APP_H

#include "robot_imu.h"
#include "robot_battery.h"
#include "robot_app.h"

#include "ldlidar_driver/ldlidar_datatype.h"

#include <unordered_map>

typedef struct RobotStatus
{
    ImuStatus *imuStatus;
    BatteryStatus *batteryStatus;
    bool systemError;
} RobotStatus;

typedef struct RobotState
{
    rc_mpu_data_t *imuData;
    std::unordered_map<int, ldlidar::PointData> *lidarMap;
} RobotState;

extern uint8_t running;

extern pthread_mutex_t robotStatusMutex;
extern RobotStatus robotStatus;

extern pthread_mutex_t imuDataMutex;
extern rc_mpu_data_t imuData;

extern pthread_mutex_t robotStateMutex;
extern RobotState robotState;

#endif