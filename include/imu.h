#include <rc/mpu.h>

rc_mpu_data_t imuData;

typedef struct ImuStatus
{
    bool accelError;
    bool initError;
    bool gyroError;
} ImuStatus;
