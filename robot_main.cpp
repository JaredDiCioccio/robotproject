////////////////////////////
// STD HEADERS
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <unordered_map>
#include <iostream>
#include <fstream>
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
#include "ini.h"
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
static pthread_t ledUpdaterThread;
static pthread_t statusUpdaterThread;

pthread_mutex_t imuDataMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t lidarDataMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t robotStatusMutex = PTHREAD_MUTEX_INITIALIZER;

////////////////////////////

////////////////////////////
// Application info
const char *message = "Hello robotics project";
uint8_t running;
RobotStatus robotStatus;
RobotState robotState;
static OperationalState currentOperationalState;
static OperationalState nextOperationalState;
static OperationalState savedOperationalState;
RobotConfiguration robotConfiguration;

std::unordered_map<OperationalState, std::string> stateLabels;
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

#define US_DELAY 250000
void *ledHandler(void *unused)
{
    while (rc_get_state() != EXITING)
    {
        while (currentOperationalState == STOPPED || currentOperationalState == IDLE && rc_get_state() != EXITING)
        {
            rc_led_set(RC_LED_GREEN, 0);
            rc_led_set(RC_LED_RED, 0);
            rc_usleep(US_DELAY);
            rc_led_set(RC_LED_RED, 1);
            rc_usleep(US_DELAY);
        }

        while (currentOperationalState == MOVING_FORWARD && rc_get_state() != EXITING)
        {
            rc_led_set(RC_LED_RED, 0);
            rc_led_set(RC_LED_GREEN, 1);
        }

        while (currentOperationalState == SCANNING && rc_get_state() != EXITING)
        {
            rc_led_set(RC_LED_GREEN, 1);
            rc_usleep(US_DELAY);
            rc_led_set(RC_LED_GREEN, 0);
            rc_usleep(US_DELAY);
        }
    }

    pthread_exit(NULL);
}

void *statusUpdater(void *unused)
{
    while (rc_get_state() != EXITING)
    {
        spdlog::info("Robot Status: ");
        spdlog::info("IMU data: Heading {0}, Gyro {1}, {2}, {3}", robotState.imuData->gyro[0], robotState.imuData->gyro[1], robotState.imuData->gyro[2]);
        spdlog::info("IMU data: Accel  {0}, {1}, {2}",
                     robotState.imuData->accel[0], robotState.imuData->accel[1], robotState.imuData->accel[2]); // std::unordered_map<int, ldlidar::PointData>::iterator it;
        spdlog::info("Battery Status: Pack Voltage: {0}V", robotStatus.batteryStatus->pack_voltage);
        // for (it = robotState.lidarMap->begin(); it != robotState.lidarMap->end(); it++)
        // {
        //     int angle = it->first;
        //     ldlidar::PointData pointData = it->second;
        //     uint64_t current = GetTimestamp();
        //     uint64_t delta = current - pointData.stamp;
        //     double age = delta / 1000000000.0;
        //     spdlog::info("Angle {0} -> {1}, age {2}", pointData.angle, pointData.distance, age);
        // }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    pthread_exit(NULL);
}

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

    bool foundLidar = false;
    // Wait until LIDAR IS AVAILABLE
    rc_led_set(RC_LED_RED, 1);
    rc_led_set(RC_LED_GREEN, 1);
    int ledState = 1;
    while (!foundLidar && rc_get_state() != EXITING)
    {
        std::ifstream file;
        file.open("/dev/ttyUSB0");
        if (file)
        {
            spdlog::info("Found lidar");
            foundLidar = true;
        }
        else
        {
            spdlog::warn("Could not find lidar...");
            ledState ^= 1;
            rc_led_set(RC_LED_RED, ledState);
            rc_led_set(RC_LED_GREEN, ledState);
        }
        rc_usleep(500000);
    }

    if (rc_get_state() == EXITING)
    {
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
    if (robotConfiguration.motorRunTest)
    {

        spdlog::info("Testing motors Forward");
        moveForward(robotConfiguration.motorSpeedForward);
        sleep(2);
        stop();
        sleep(0.5);
        spdlog::info("Testing motors Backward");
        moveBackward(robotConfiguration.motorSpeedBackward);
        sleep(2);
        stop();
        sleep(0.5);
        spdlog::info("Testing motors Turn Left");
        turnLeft(robotConfiguration.motorSpeedTurn);
        sleep(2);
        stop();
        sleep(0.5);
        spdlog::info("Testing motors Turn Right");
        turnRight(robotConfiguration.motorSpeedTurn);
        sleep(2);
        stop();
    }
    robot_lidar_init(&robotState);
    spdlog::info("Initialized lidar");

    robot_imu_init(&robotState);
    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();
    spdlog::trace("robot_main.cpp startup() return");
    return 0;
}

// https://docs.oracle.com/cd/E19455-01/806-5257/attrib-16/index.html
pthread_attr_t makePriorityParams(int newprio)
{
    pthread_attr_t tattr;
    int ret;
    sched_param param;

    ret = pthread_attr_init(&tattr);
    ret = pthread_attr_getschedparam(&tattr, &param);
    param.sched_priority = newprio;
    ret = pthread_attr_setschedparam(&tattr, &param);
    return tattr;
}

void parseIni(std::string &iniFile)
{
    mINI::INIFile file(iniFile);
    mINI::INIStructure ini;
    spdlog::info("Parsing ini file. {0}", iniFile);
    file.read(ini);
    spdlog::info("Parsing ini file. Done");

    float backwardDuty = std::stof(ini["motor"]["backwardDuty"]);
    float turnDuty = std::stof(ini["motor"]["turnDuty"]);
    float forwardDuty = std::stof(ini["motor"]["forwardDuty"]);
    int stopThreshold = std::stoi(ini["lidar"]["stopThreshold"]);
    bool runMotorTest = ini["motor"].has("startupTest");
    robotConfiguration.motorSpeedBackward = backwardDuty;
    robotConfiguration.motorSpeedForward = forwardDuty;
    robotConfiguration.motorSpeedTurn = turnDuty;
    robotConfiguration.stopThreshold = stopThreshold;
    robotConfiguration.motorRunTest = runMotorTest;

    spdlog::info("Motor Speed Forward: {0}", forwardDuty);

    spdlog::info("Motor Speed Backward: {0}", backwardDuty);
    spdlog::info("Motor Speed Turn: {0}", turnDuty);
    spdlog::info("Stop Threshold {0}", stopThreshold);
}

int main(int argc, char **args)
{
    const auto startupTime = std::chrono::steady_clock::now();
    std::string iniFile = "robot_configuration.ini";
    if (argc > 1)
    {
        iniFile = args[1];
    }

    parseIni(iniFile);

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

    stateLabels[MOVING_FORWARD] = "Moving Forward";
    stateLabels[TURNING_LEFT] = "Turning Left";
    stateLabels[TURNING_RIGHT] = "Turning Right";
    stateLabels[SCANNING] = "Scanning";
    stateLabels[STOPPED] = "Stopped";
    stateLabels[IDLE] = "Idle";
    currentOperationalState = STOPPED;

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
    pthread_create(&imuUpdateThread, NULL, imu_updater, NULL);
    pthread_create(&lidarUpdateThread, NULL, robot_lidar_updater, NULL);
    pthread_create(&ledUpdaterThread, NULL, ledHandler, NULL);
    pthread_create(&statusUpdaterThread, NULL, statusUpdater, NULL);
    pthread_create(&batteryUpdateThread, NULL, batteryStatusUpdater, NULL);

    spdlog::debug("Setting inital state to PAUSED");

    rc_set_state(PAUSED); // Start paused until the pause button is pressed

    spdlog::debug("Entering loop");
    std::unordered_map<int, ldlidar::PointData> *map = robotState.lidarMap;
    spdlog::debug("Grabbed reference to lidar map");

    while (rc_get_state() != EXITING)
    {
        // spdlog::debug("Current State = {0}", stateLabels[currentOperationalState]);
        uint64_t currentTimestamp = GetTimestamp();

        if (currentOperationalState == MOVING_FORWARD)
        {
            // spdlog::debug("In MOVING_FORWARD");
            bool shouldStop = !fovIsClear(30);

            if (shouldStop)
            {
                spdlog::info("Stopping motors due to obstruction");
                stopMotors();
                nextOperationalState = TURNING_LEFT;
            }
            else
            {
                // spdlog::debug("Moving forward");
                moveForward(robotConfiguration.motorSpeedForward);
            }
        }

        if (currentOperationalState == STOPPED)
        {
            stopMotors();
            nextOperationalState = IDLE;
        }

        if (currentOperationalState == SCANNING)
        {
            if (fovIsClear(30))
            {
                nextOperationalState = MOVING_FORWARD;
            }
        }

        if (currentOperationalState == TURNING_LEFT)
        {
            turnLeft(robotConfiguration.motorSpeedTurn);
            nextOperationalState = SCANNING;
        }

        if (nextOperationalState != currentOperationalState)
        {
            spdlog::info("STATE CHANGE {0} -> {1}", stateLabels[currentOperationalState], stateLabels[nextOperationalState]);
            currentOperationalState = nextOperationalState;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    spdlog::info("Waiting for threads to exit...");
    pthread_join(lidarUpdateThread, NULL);
    spdlog::info("Lidar thread exited");
    pthread_join(ledUpdaterThread, NULL);
    spdlog::info("LED thread exited");
    pthread_join(statusUpdaterThread, NULL);
    spdlog::info("Status update thread exited");
    pthread_join(imuUpdateThread, NULL);
    spdlog::info("IMU thread exited");
    pthread_join(batteryUpdateThread, NULL);
    spdlog::info("Battery thread exited");

    spdlog::info("All threads exited. Cleaning up...");

    stopMotors();
    rc_motor_cleanup();
    rc_gpio_cleanup(RC_BTN_PIN_PAUSE);
    spdlog::info("Exiting");
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
        savedOperationalState = currentOperationalState;
        nextOperationalState = STOPPED;
    }
    else if (rc_get_state() == PAUSED)
    {
        spdlog::info("Changing from PAUSED -> RUNNING");
        rc_set_state(RUNNING);
        nextOperationalState = savedOperationalState;
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
