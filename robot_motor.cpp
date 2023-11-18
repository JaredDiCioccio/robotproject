#include "robot_motor.h"
#include "rc/motor.h"

MotorConfiguration leftMotorConfig ={.channel=LEFT_MOTOR_CHANNEL, .polarity=LEFT_MOTOR_POLARITY};
MotorConfiguration rightMotorConfig ={.channel=RIGHT_MOTOR_CHANNEL, .polarity=RIGHT_MOTOR_POLARITY};


void robot_motor_init()
{
    rc_motor_init();
}

void stopMotors(){
    stop();
}
void stop()
{
    rc_motor_set(LEFT_MOTOR_CHANNEL, 0);
    rc_motor_set(RIGHT_MOTOR_CHANNEL, 0);
}

void moveForward()
{
    rc_motor_set(leftMotorConfig.channel, 0.15 * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, 0.15 * rightMotorConfig.polarity);
}

void moveBackward()
{
    rc_motor_set(leftMotorConfig.channel, -0.15 * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, -0.15 * rightMotorConfig.polarity);
}

void turnLeft()
{
    stop();
    rc_motor_set(leftMotorConfig.channel, -0.15 * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, 0.15 * rightMotorConfig.polarity);
}

void turnRight()
{
    stop();
    rc_motor_set(leftMotorConfig.channel, 0.15 * leftMotorConfig.polarity);
    rc_motor_set(rightMotorConfig.channel, -0.15 * rightMotorConfig.polarity);
}