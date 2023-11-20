#ifndef ROBOT_LIDAR_H
#define ROBOT_LIDAR_H

#include "robot_app.h"

void robot_lidar_init(RobotState *robotState);
void *robot_lidar_updater(void *unused);
uint64_t GetTimestamp(void);
bool fovIsClear();
// Input full fov. Will scan +- fov/2 in either direction.
bool fovIsClear(int fov);
bool fovIsClear(int centerAngle, int fov);

#endif