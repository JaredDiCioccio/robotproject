////////////////////////////
// STD HEADERS
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <unordered_map>
// SYSTEM HEADERS
#include <pthread.h>
#include <signal.h>
////////////////////////////
// ROBOTICS LIB HEADERS
#include <robotcontrol.h>
////////////////////////////
// THIRD PARTY LIBS
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "ldlidar_driver/ldlidar_datatype.h"
////////////////////////////
// MY HEADERS
#include "robot_app.h"
#include "robot_imu.h"
#include "robot_battery.h"
#include "robot_motor.h"
#include "robot_lidar.h"
////////////////////////////

////////////////////////////
// Threads
static pthread_t loopThread;
static pthread_t errorHandlerThread;
static pthread_t imuUpdateThread;
static pthread_t batteryUpdateThread;
static pthread_t lidarUpdateThread;

pthread_mutex_t imuDataMutex = PTHREAD_MUTEX_INITIALIZER;
////////////////////////////

////////////////////////////
// Application info
const char *message = "Hello robotics project";
uint8_t running;
pthread_mutex_t robotStatusMutex = PTHREAD_MUTEX_INITIALIZER;
RobotStatus robotStatus;
RobotState robotState;

#define STOP_THRESHOLD 500

//////////////////////

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
    spdlog::info("Exiting");
    rc_set_state(EXITING);
    return;
}

// function declarations
void on_pause_press();
void on_pause_release();
int startup();

void *errorHandler(void *unused)
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
            spdlog::error("System Error.Exiting");
            running = false;
        }

        if (localStatus.imuStatus->initError)
        {
            spdlog::error("IMU Error. Exiting");
            running = false;
        }
    }
    pthread_exit(NULL);
}

// Most of this borrowed from robot control lib template
int startup()
{
    spdlog::trace("robot_main.cpp startup()");
    if (rc_kill_existing_process(2.0) < -2)
        return -1;

    // start signal handler so we can exit cleanly
    if (rc_enable_signal_handler() == -1)
    {
        spdlog::error("ERROR: failed to start signal handler");
        return -1;
    }

    // initialize pause button
    if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                       RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        spdlog::error("ERROR: failed to initialize pause button");
        return -1;
    }

    // Assign functions to be called when button events occur
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, on_pause_release);

    int ret = robot_motor_init();
    if (ret != 0)
    {
        spdlog::error("ERROR: failed to initialize motors");
        return -1;
    }
    spdlog::info("Initialized motors");
    spdlog::info("Testing motors Forward");
    moveForward();
    sleep(2);
    spdlog::info("Testing motors Backward");
    moveBackward();
    sleep(2);
    stop();
    robot_lidar_init(&robotState);
    spdlog::info("Initialized lidar");
    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();
    spdlog::trace("robot_main.cpp startup() return");
    return 0;
}

int main(int argc, char **args)
{
    const auto startupTime = std::chrono::system_clock::now();
    // int epochCount = startupTime.time_since_epoch().count();
    // std::string filename = "robot_log_" + std::to_string(epochCount) + ".log";
    // std::vector<spdlog::sink_ptr> sinks;
    // sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_st>());
    // sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_st>(filename));
    // auto combined_logger = std::make_shared<spdlog::logger>("RobotLogger", std::begin(sinks), std::end(sinks));
    // register it if you need to access it globally
    // spdlog::register_logger(combined_logger);
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("Starting Robotics Project");

    int ret = startup();
    if (ret != 0)
    {
        spdlog::error("Error starting up robot!");
        exit(1);
    }
    signal(SIGINT, __signal_handler);
    spdlog::debug("Registered signal handler");
    // pthread_create(&loopThread, NULL, loop, NULL);
    // pthread_create(&errorHandlerThread, NULL, errorHandler, NULL);
    // pthread_create(&imuUpdateThread, NULL, imu_updater, NULL);
    pthread_create(&lidarUpdateThread, NULL, robot_lidar_updater, NULL);

    spdlog::debug("Setting inital state to PAUSED");

    rc_set_state(PAUSED); // Start paused until the pause button is pressed

    spdlog::debug("Entering loop");
    std::unordered_map<int, ldlidar::PointData> *map = robotState.lidarMap;
    spdlog::debug("Grabbed reference to lidar map");
    while (rc_get_state() != EXITING)
    {
        auto currentTime = std::chrono::system_clock::now().time_since_epoch().count();
        spdlog::debug("Tick: {0}", currentTime);
        bool shouldStop = false;
        for (int i = 345; i < 360 && !shouldStop; i++)
        {
            if (map->count(i))
            {
                ldlidar::PointData pointData = map->at(i);
                if (pointData.stamp > (currentTime - 50000000)) // 50 millsecond falloff?
                {
                    spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTime, currentTime - pointData.stamp, i, pointData.distance);
                    map->erase(i);
                }
                else
                {
                    if (pointData.distance < STOP_THRESHOLD)
                    {
                        spdlog::info("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
                        shouldStop = true;
                    }
                }
            }
        }

        for (int i = 0; i < 15 && !shouldStop; i++)
        {
            if (map->count(i))
            {
                ldlidar::PointData pointData = map->at(i);
                if (pointData.stamp > (currentTime - 50000000)) // 50 millsecond falloff?
                {
                    spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTime, currentTime - pointData.stamp, i, pointData.distance);
                    map->erase(i);
                }
                else
                {
                    if (pointData.distance < STOP_THRESHOLD)
                    {
                        spdlog::info("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
                        shouldStop = true;
                    }
                }
            }
        }

        if (shouldStop || rc_get_state() == PAUSED)
        {
            if (shouldStop)
            {
                spdlog::info("Stopping because we should stop");
            }
            if (rc_get_state() == PAUSED)
            {
                spdlog::info("Sopping because we're paused");
            }

            stopMotors();
        }
        else
        {
            moveForward();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rc_motor_cleanup();
    rc_gpio_cleanup(RC_BTN_PIN_PAUSE);
    spdlog::info("Waiting for threads to exit...");
    pthread_join(lidarUpdateThread, NULL);

    spdlog::info("All threads exited. Exiting...");
    exit(0);
}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
    if (rc_get_state() == RUNNING)
    {
        spdlog::info("Changing from RUNNING -> PAUSED");
        rc_set_state(PAUSED);
    }
    else if (rc_get_state() == PAUSED)
    {
        spdlog::info("Changing from PAUSED -> RUNNING");
        rc_set_state(RUNNING);
    }
    return;
}

/**
 * If the user holds the pause button for 2 seconds, set state to EXITING which
 * triggers the rest of the program to exit cleanly.
 **/
void on_pause_press()
{
    int i;
    const int samples = 100;     // check for release 100 times in this period
    const int us_wait = 2000000; // 2 seconds

    // now keep checking to see if the button is still held down
    for (i = 0; i < samples; i++)
    {
        rc_usleep(us_wait / samples);
        if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED)
            return;
    }
    spdlog::info("long press detected, shutting down");
    rc_set_state(EXITING);
    return;
}
