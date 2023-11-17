
#include "rc/motor.h"

typedef struct CurrentMotorState{
    float rightMotorDuty;
    float leftMotorDuty;
} CurrentMotorState;

void init(){
    rc_motor_init();
}

void stopMotors();
void setMotorLeft(float duty);
void setMotorRight(float duty);
