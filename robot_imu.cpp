////////////////////////////
// ROBOTICS LIB HEADERS
#include <rc/mpu.h>
#include <rc/time.h>
#include <rc/start_stop.h>
////////////////////////////
////////////////////////////
// THIRD PARTY LIBS
#include "spdlog/spdlog.h"
////////////////////////////
// MY HEADERS
#include "robot_imu.h"
#include "robot_app.h"
////////////////////////////

// I2C STUFF
#define I2C_BUS 2
rc_mpu_data_t imuData;
ImuStatus imuStatus;
///////////////////

extern pthread_mutex_t robotStatusMutex;

void robot_imu_init(RobotState *robotState)
{
    robotState->imuData = &imuData;
}

void *imu_updater(void *unused)
{
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.enable_magnetometer = 1;
    conf.show_warnings = 0;
    robotStatus.imuStatus = &imuStatus;

    if (rc_mpu_initialize(&imuData, conf))
    {
        spdlog::error("rc_mpu_initialize_failed");
        pthread_mutex_lock(&robotStatusMutex);
        imuStatus.initError = 1;
        pthread_mutex_lock(&robotStatusMutex);
        rc_set_state(EXITING);
    }

    int accelStatus = 0;
    int gyroStatus = 0;
    int error = 0;
    while (rc_get_state() != EXITING)
    {

        accelStatus = 0;
        gyroStatus = 0;
        error = 0;

        pthread_mutex_lock(&imuDataMutex);
        accelStatus = rc_mpu_read_accel(&imuData);
        gyroStatus = rc_mpu_read_gyro(&imuData);
        rc_mpu_read_mag(&imuData);
        rc_mpu_read_temp(&imuData);
        
        pthread_mutex_unlock(&imuDataMutex);

        if (accelStatus < 0)
        {
            pthread_mutex_lock(&robotStatusMutex);
            imuStatus.accelError = 1;
            pthread_mutex_unlock(&robotStatusMutex);

            error = 1;
            spdlog::error("Error reading IMU Accel Data");
        }
        if (gyroStatus < 0)
        {
            pthread_mutex_unlock(&robotStatusMutex);
            imuStatus.gyroError = 1;
            pthread_mutex_unlock(&robotStatusMutex);

            error = 1;
            spdlog::error("Error reading IMU Gyro Data");
        }

        rc_usleep(1000000 / 100);
    }

    return nullptr;
}