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

// Must be here to define i2c bus for imu
#define I2C_BUS 2

extern const char *message;

static pthread_t loopThread;
static uint8_t running = 1;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
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
    signal(SIGINT,__signal_handler);
    pthread_create(&loopThread, NULL, loop, NULL);

    while (running)
    {
        sleep(1);
    }
    log_info("Waiting for threads to exit...");
    pthread_join(loopThread, NULL);

    log_info("All threads exited. Exiting...");
    exit(0);
}