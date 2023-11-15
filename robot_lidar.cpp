#include <chrono>
#include <thread>
#include <string>

#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

// In typical fashion this has to be included before ldlidar_driver_linux.h
#include "spdlog/spdlog.h"

#include "ldlidar_driver/ldlidar_driver_linux.h"

// Note, some of these functions taken from the ldlidar demo application

uint64_t GetTimestamp(void)
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

void *lidarUpdater(void *unused)
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
        // spdlog::info("ldlidar serial connect is success");
        // LidarPowerOn();
    }
    else
    {
        // spdlog::error("ldlidar serial connect is fail");
        // exit(EXIT_FAILURE);
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

    while (ldlidar::LDLidarDriverLinuxInterface::Ok())
    {
        switch (lidar_drv->GetLaserScanData(laser_scan_points, 1500))
        {
        case ldlidar::LidarStatus::NORMAL:
        {
            double lidar_scan_freq = 0;
            lidar_drv->GetLidarScanFreq(lidar_scan_freq);
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp;

            for (auto point : laser_scan_points)
            {
                point.stamp, point.angle, point.distance, point.intensity;
            }
            break;
        }
        case ldlidar::LidarStatus::DATA_TIME_OUT:
        {
            lidar_drv->Stop();
            break;
        }
        case ldlidar::LidarStatus::DATA_WAIT:
        {
            break;
        }
        default:
            break;
        }
    }
    return (void *)1;
}