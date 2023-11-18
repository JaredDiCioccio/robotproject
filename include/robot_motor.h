#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H


#include "rc/motor.h"
#include <signal.h>

typedef struct CurrentMotorState{
    float rightMotorDuty;
    float leftMotorDuty;
} CurrentMotorState;

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 2

#define LEFT_MOTOR_POLARITY 1
#define RIGHT_MOTOR_POLARITY -1

struct MotorConfiguration{
    int channel;
    int polarity;
};

void robot_motor_init();
void stop();
void stopMotors();
void moveForward();
void moveBackward();
void setMotorLeft(float duty);
void setMotorRight(float duty);
void turnLeft();
void turnRight();


#endif