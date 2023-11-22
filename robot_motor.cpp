#include "robot_motor.h"
#include "rc/motor.h"

MotorConfiguration leftMotorConfig = {.channel = LEFT_MOTOR_CHANNEL, .polarity = LEFT_MOTOR_POLARITY};
MotorConfiguration rightMotorConfig = {.channel = RIGHT_MOTOR_CHANNEL, .polarity = RIGHT_MOTOR_POLARITY};

int robot_motor_init()
{
    int ret = rc_motor_init();
    return ret;
}

void stopMotors()
{
    stop();
}
void stop()
{
    rc_motor_set(LEFT_MOTOR_CHANNEL, 0);
    rc_motor_set(RIGHT_MOTOR_CHANNEL, 0);
}

void moveForward(float duty)
{
    rc_motor_set(leftMotorConfig.channel, duty * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, duty * rightMotorConfig.polarity);
}

void moveBackward(float duty)
{
    rc_motor_set(leftMotorConfig.channel, -duty * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, -duty * rightMotorConfig.polarity);
}

void turnLeft(float duty)
{
    rc_motor_set(leftMotorConfig.channel, duty * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, -duty * 1.10 * rightMotorConfig.polarity);
}

void turnRight(float duty)
{
    rc_motor_set(leftMotorConfig.channel, -duty * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, duty * rightMotorConfig.polarity);
}