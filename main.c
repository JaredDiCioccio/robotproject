////////////////////////////
// STD HEADERS
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
// SYSTEM HEADERS
#include <pthread.h>
#include <signal.h>
////////////////////////////
// ROBOTICS LIB HEADERS
#include <rc/mpu.h>
#include <rc/time.h>
////////////////////////////
// THIRD PARTY LIBS
#include "log.h"
////////////////////////////
// MY HEADERS
#include "app.h"
////////////////////////////

////////////////////////////
// Threads
static pthread_t loopThread;
static pthread_t errorHandlerThread;

////////////////////////////

////////////////////////////
// Application info
const char *message = "Hello robotics project";
uint8_t running;
pthread_mutex_t robotStatusMutex = PTHREAD_MUTEX_INITIALIZER;
RobotStatus robotStatus;

pthread_mutex_t imuDataMutex = PTHREAD_MUTEX_INITIALIZER;
rc_mpu_data_t imuData;
//////////////////////

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

void errorHandler(void *unused)
{
    RobotStatus localStatus;
    while (running)
    {

        // Make a local copy so we don't waste time holding the mutex
        // For logic
        pthread_mutex_lock(&robotStatusMutex);
        localStatus = robotStatus;
        pthread_mutex_unlock(&robotStatusMutex);

        if (localStatus.systemError == 1)
        {
            log_error("System Error.Exiting");
            running = false;
        }

        if (localStatus.imuStatus->initError)
        {
            log_error("IMU Error. Exiting");
            running = false;
        }
    }
}

void *loop()
{
    static int counter = 0;
    while (running)
    {
        log_info("[%d] %s", counter++, message);
        sleep(1);
    }
}
int main(int argc, char **args)
{
    log_info("Starting Robotics Project");
    signal(SIGINT, __signal_handler);
    pthread_create(&loopThread, NULL, loop, NULL);
    pthread_create(&errorHandlerThread, NULL, loop, NULL);

    running = 1;
    while (running)
    {
        sleep(1);
    }
    log_info("Waiting for threads to exit...");
    pthread_join(loopThread, NULL);

    log_info("All threads exited. Exiting...");
    exit(0);
}