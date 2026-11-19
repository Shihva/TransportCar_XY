#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "Moto.h"
#include "pid.h"

typedef struct
{
    SingleMotor motor[4];
		PID Yaw;

    struct 
	{
		float wheeltrack;
        float wheelbase;
		float wheelradius;
	} info;

    struct Move
	{
		float vx;//前正//右手系，拇指朝上z
		float vy;//左正
		float vw;//rad/s
		float yawAngle;//rad

		float maxVx, maxVy, maxVw, yawDEADZONE;//Vw rad/s

		float vx_real, vy_real, vw_real, yawAngle_real;
	} move;

	enum
	{
		Chassis_Control,
		Chassis_AI,
	} pattern;
}Chassis;

extern Chassis chassis;
void Chassis_GetBodyWheelRpm(float rpm[4]);

#endif
