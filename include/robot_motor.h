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


#define DEFAULT_TURNING_DUTY 0.10
#define DEFAULT_FORWARD_DUTY 0.35
#define DEFAULT_BACKWARD_DUTY -0.25

struct MotorConfiguration{
    int channel;
    int polarity;
};

int robot_motor_init();
void stop();
void stopMotors();

void moveForward(float duty);
void moveBackward(float duty);
void turnLeft(float duty);
void turnRight(float duty);

void setMotorLeft(float duty);
void setMotorRight(float duty);


#endif