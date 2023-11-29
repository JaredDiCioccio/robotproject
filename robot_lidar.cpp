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
extern pthread_mutex_t lidarDataMutex;
uint64_t GetTimestamp(void)
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

#define POINT_DATA_TTL 250000000

void robot_lidar_init(RobotState *robotState)
{
    robotState->lidarData = &pointsBuffer;
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
            uint64_t currentTimestamp = GetTimestamp();
            pthread_mutex_lock(&lidarDataMutex);
            auto itr = pointsBuffer.begin();
            while (itr != pointsBuffer.end() && rc_get_state() != EXITING)
            {
                uint64_t age = currentTimestamp - itr->stamp;

                if (age > POINT_DATA_TTL)
                {
                    itr = pointsBuffer.erase(itr);
                }
                else
                {
                    itr++;
                }
            }

            for (auto pointData : laser_scan_points)
            {

                // uint64_t age = GetTimestamp() - pointData.stamp;
                // spdlog::debug("PointData {0}: {1} deg, {2} dist, {3} intensity, {4} age ",
                //               pointData.stamp,
                //               pointData.angle,
                //               pointData.distance,
                //               pointData.intensity,
                //               age);
                if (pointData.intensity >= 200)
                {
                    pointsBuffer.push_back(pointData);
                }
                else
                {
                    // badPointsString += std::to_string(pointData.angle) + ":" + std::to_string(pointData.intensity) + ",";
                }
            }
            // spdlog::info(badPointsString);

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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    lidar_drv->Stop();
    pthread_exit(NULL);
}

// This does not handle 0 crossing
// bool fovIsClear(int centerAngle, int fov)
// {
//     uint64_t currentTimestamp = GetTimestamp();

//     if (centerAngle < 360)
//     {
//         centerAngle = 360 + centerAngle;
//         // StartAngle = -60
//         // StartAngle -> 360 + 60 = 300
//     }
//     if (fov % 2 != 0)
//     {
//         fov += 1;
//     }

//     int halfFov = fov / 2;

//     // startAngle =300, fov = 30
//     // Check from 285 - 315

//     for(auto pointData : pointsBuffer){

//     }
//     for (int i = centerAngle - halfFov; i < centerAngle + halfFov; i++)
//     {
//         if (pointsMap.count(i))
//         {
//             ldlidar::PointData pointData = pointsMap.at(i);
//             spdlog::debug("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
//             if (currentTimestamp - pointData.stamp > 250000000) // 250 millsecond falloff?
//             {
//                 spdlog::debug("Erasing old data Old stamp: {0}, current time {1}, delta {2}. Angle {3}->{4}", pointData.stamp, currentTimestamp, currentTimestamp - pointData.stamp, i, pointData.distance);
//                 pointsMap.erase(i);
//             }
//             else
//             {
//                 if (pointData.distance < robotConfiguration.stopThreshold)
//                 {
//                     spdlog::info("Timestamp: {2}: \tAngle {0} -> {1}", pointData.angle, pointData.distance, pointData.stamp);
//                     return false;
//                 }
//             }
//         }
//     }
//     return true;
// }

bool fovIsClear()
{
    return fovIsClear(30);
}

// Input full fov. Will scan +- fov/2 in either direction.
bool fovIsClear(int fov)
{
    pthread_mutex_lock(&lidarDataMutex);
    uint64_t currentTimestamp = GetTimestamp();
    if (pointsBuffer.size() < 5)
    {
        return false;
    }
    bool clear = true;
    if (fov % 2 != 0)
    {
        fov += 1;
    }

    int halfFov = fov / 2;
    auto itr = pointsBuffer.begin();
    spdlog::debug("Checking {1} points for if fov is clear {0}", currentTimestamp, pointsBuffer.size());
    while (itr != pointsBuffer.end() && rc_get_state() != EXITING)
    {
        uint64_t age = currentTimestamp - itr->stamp;

        if (!(itr->angle >= 360 - halfFov || itr->angle <= halfFov))
        {
            itr = pointsBuffer.erase(itr);
        }
        else if (age > POINT_DATA_TTL)
        {
            spdlog::debug("Removing angle {0} due to age {1} - {2} = {3}", itr->angle, currentTimestamp, itr->stamp, age);
            itr = pointsBuffer.erase(itr);
        }
        else
        {
            if (itr->distance <= robotConfiguration.stopThreshold)
            {
                spdlog::info("Found obstacle at {0} deg, {1} distance", itr->angle, itr->distance);
                clear = false;
            }
            itr++;
        }
    }
    spdlog::debug("FOV is clear: {0}", clear);
    pthread_mutex_unlock(&lidarDataMutex);
    return clear;
}
