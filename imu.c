////////////////////////////
// ROBOTICS LIB HEADERS
#include <rc/mpu.h>
#include <rc/time.h>
////////////////////////////
////////////////////////////
// THIRD PARTY LIBS
#include "log.h"
////////////////////////////
// MY HEADERS
#include "app.h"
////////////////////////////

// I2C STUFF
#define I2C_BUS 2
rc_mpu_data_t imuData;
ImuStatus imuStatus;
///////////////////

extern pthread_mutex_t robotStatusMutex;

void *imu_updater()
{
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.enable_magnetometer = 0;
    conf.show_warnings = 0;
    robotStatus.imuStatus = &imuStatus;

    if (rc_mpu_initialize(&imuData, conf))
    {
        log_error("rc_mpu_initialize_failed");
        pthread_mutex_lock(&robotStatusMutex);
        imuStatus.initError = 1;
        pthread_mutex_lock(&robotStatusMutex);
        return -1;
    }

    int accelStatus = 0;
    int gyroStatus = 0;
    int error = 0;
    while (running)
    {

        accelStatus = 0;
        gyroStatus = 0;
        error = 0;

        pthread_mutex_lock(&imuDataMutex);
        accelStatus = rc_mpu_read_accel(&imuData);
        gyroStatus = rc_mpu_read_gyro(&imuData);
        pthread_mutex_unlock(&imuDataMutex);

        if (accelStatus < 0)
        {
            pthread_mutex_lock(&robotStatusMutex);
            imuStatus.accelError = 1;
            pthread_mutex_unlock(&robotStatusMutex);

            error = 1;
            log_error("Error reading IMU Accel Data");
        }
        if (gyroStatus < 0)
        {
            pthread_mutex_unlock(&robotStatusMutex);
            imuStatus.gyroError = 1;
            pthread_mutex_unlock(&robotStatusMutex);

            error = 1;
            log_error("Error reading IMU Gyro Data");
        }
    }
}