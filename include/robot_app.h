#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "ldlidar_driver/ldlidar_datatype.h"
#include <rc/mpu.h>

#include <unordered_map>

typedef struct BatteryStatus
{
    double pack_voltage; // 2S pack voltage on JST XH 2S balance connector
    double cell_voltage; // cell voltage
    double jack_voltage; // could be dc power supply or another battery
    bool error;
} BatteryStatus;

typedef struct ImuStatus
{
    bool accelError;
    bool initError;
    bool gyroError;
} ImuStatus;

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

enum OperationalState
{
    MOVING_FORWARD,
    STOPPED,
    TURNING_LEFT,
    TURNING_RIGHT,
    SCANNING,
    IDLE
};

extern pthread_mutex_t robotStatusMutex;
extern RobotStatus robotStatus;

extern pthread_mutex_t imuDataMutex;
extern rc_mpu_data_t imuData;

extern pthread_mutex_t robotStateMutex;
extern RobotState robotState;

#define STOP_THRESHOLD 300

#endif