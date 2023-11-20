#include <chrono>
#include <thread>
#include <string>

#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

// In typical fashion this has to be included before ldlidar_driver_linux.h
#include "spdlog/spdlog.h"
#include <unordered_map>

#include "ldlidar_driver/ldlidar_driver_linux.h"
#include "robot_app.h"
#include "robot_lidar.h"
// Note, some of these functions taken from the ldlidar demo application

ldlidar::Points2D pointsBuffer;
std::unordered_map<int, ldlidar::PointData> pointsMap;
extern pthread_mutex_t lidarDataMutex;
uint64_t GetTimestamp(void)
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

void robot_lidar_init(RobotState *robotState)
{
    robotState->lidarMap = &pointsMap;
}

void *robot_lidar_updater(void *unused)
{

    std::string ldlidar_type_str = "LD19";
    std::string serial_port_name = "/dev/ttyUSB0";
    uint32_t serial_baudrate_val = 230400;
    ldlidar::LDType ldlidar_type_dest = ldlidar::LDType::LD_19;

    ldlidar::LDLidarDriverLinuxInterface *lidar_drv = ldlidar::LDLidarDriverLinuxInterface::Create();
    lidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp));
    lidar_drv->EnablePointCloudDataFilter(true);

    if (lidar_drv->Connect(ldlidar_type_dest, serial_port_name, serial_baudrate_val))
    {
        spdlog::info("ldlidar serial connect is success");
        // LidarPowerOn();
    }
    else
    {
        spdlog::error("ldlidar serial connect is fail");
        exit(EXIT_FAILURE);
    }

    if (lidar_drv->WaitLidarComm(3500))
    {
    }
    else
    {
        lidar_drv->Disconnect();
    }

    if (lidar_drv->Start())
    {
    }
    else
    {
    }

    ldlidar::Points2D laser_scan_points;
    spdlog::info("Entering lidar loop");
    while (ldlidar::LDLidarDriverLinuxInterface::Ok() && rc_get_state() != EXITING)
    {
        switch (lidar_drv->GetLaserScanData(laser_scan_points, 1500))
        {
        case ldlidar::LidarStatus::NORMAL:
        {
            double lidar_scan_freq = 0;
            lidar_drv->GetLidarScanFreq(lidar_scan_freq);
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp;

            pthread_mutex_lock(&lidarDataMutex);
            for (auto point : laser_scan_points)
            {
                point.stamp, point.angle, point.distance, point.intensity;
                int roundedAngle = std::round(point.angle);
                if (pointsMap.count(roundedAngle))
                {
                    pointsMap.erase(roundedAngle);
                }
                pointsMap.insert({roundedAngle, point});
            }
            pthread_mutex_unlock(&lidarDataMutex);

            break;
        }
        case ldlidar::LidarStatus::DATA_TIME_OUT:
        {
            spdlog::error("lidar timeout");
            break;
        }
        case ldlidar::LidarStatus::DATA_WAIT:
        {
            // This usually happens at startup
            spdlog::warn("lidar data wait");
            break;
        }
        default:
            spdlog::error("lidar unknow error");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(166)); // about 6 hz
    }
    lidar_drv->Stop();
    pthread_exit(NULL);
}

// This does not handle 0 crossing
bool fovIsClear(int centerAngle, int fov)
{
    uint64_t currentTimestamp = GetTimestamp();

    if (centerAngle < 360)
    {
        centerAngle = 360 + centerAngle;
        // StartAngle = -60
        // StartAngle -> 360 + 60 = 300
    }
    if (fov % 2 != 0)
    {
        fov += 1;
    }

    int halfFov = fov / 2;

    // startAngle =300, fov = 30
    // Check from 285 - 315
    for (int i = centerAngle - halfFov; i < centerAngle + halfFov; i++)
    {
        if (pointsMap.count(i))
        {
            ldlidar::PointData pointData = pointsMap.at(i);
            spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
            if (currentTimestamp - pointData.stamp > 250000000) // 250 millsecond falloff?
            {
                spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTimestamp, currentTimestamp - pointData.stamp, i, pointData.distance);
                pointsMap.erase(i);
            }
            else
            {
                if (pointData.distance < STOP_THRESHOLD)
                {
                    spdlog::info("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
                    return false;
                }
            }
        }
    }
    return true;
}

bool fovIsClear()
{
    return fovIsClear(30);
}

// Input full fov. Will scan +- fov/2 in either direction.
bool fovIsClear(int fov)
{
    uint64_t currentTimestamp = GetTimestamp();

    if (fov % 2 != 0)
    {
        fov += 1;
    }

    int halfFov = fov / 2;

    for (int i = 360 - halfFov; i < 360; i++)
    {
        if (pointsMap.count(i))
        {
            ldlidar::PointData pointData = pointsMap.at(i);
            // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
            if (currentTimestamp - pointData.stamp > 250000000) // 250 millsecond falloff?
            {
                // spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTimestamp, currentTimestamp - pointData.stamp, i, pointData.distance);
                pointsMap.erase(i);
            }
            else
            {
                if (pointData.distance < STOP_THRESHOLD)
                {
                    // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1} BAD", pointData.angle, pointData.distance, pointData.stamp);
                    return false;
                }
                else
                {
                    // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1} OK", pointData.angle, pointData.distance, pointData.stamp);
                }
            }
        }
    }

    for (int i = 0; i < halfFov; i++)
    {
        if (pointsMap.count(i))
        {

            ldlidar::PointData pointData = pointsMap.at(i);
            // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);

            if (pointData.stamp > (currentTimestamp - 250000000)) // 50 millsecond falloff?
            {
                // spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTimestamp, currentTimestamp - pointData.stamp, i, pointData.distance);
                pointsMap.erase(i);
            }
            else
            {
                if (pointData.distance < STOP_THRESHOLD)
                {
                    // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1} BAD", pointData.angle, pointData.distance, pointData.stamp);
                    return false;
                }
                else
                {
                    // spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1} OK", pointData.angle, pointData.distance, pointData.stamp);
                }
            }
        }
    }
    return true;
}