#ifndef _MOVE_H
#define _MOVE_H

#include <math.h>
#include "calculate.h"
#include "moveBase.h"
#include "MotionCard.h"
#include "timer.h"
#include "robot.h"
#include "balance.h"

void SetTargetVel(float vel,float velDir,float omega);

void VelControl(robotVel_t actVel);

//void ThreeWheelVelControl(float speed, float direction, float rotationVell);
//TriWheelVel_t CaculateThreeWheelVel(float speed, float direction, float rotationVell,float angleZ);

//TriWheelVel2_t GetTrueVell(TriWheelVel_t wheelVell, float zAngle);

//void FourWheelVelControl(float Vx, float Vy, float rotationVell);

//void VelControlTriWheel(float v1,float v2,float v3,float v4);

#endif
