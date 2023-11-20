#ifndef BATTERY_H
#define BATTERY_H

#include "robot_app.h"

void batteryStatusInit(RobotStatus *robotStatus);
void* batteryStatusUpdater(void* unused);

#endif