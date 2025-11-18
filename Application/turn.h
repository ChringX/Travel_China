#ifndef __TURN_H
#define __TURN_H

#include "sys.h"
#include "imu.h"

struct Angle {
	float AngleT;
	float AngleG;
};

float need2turn(float nowangle,float targetangle);
float getAngleZ(void);
uint8_t Turn360Step(void);
uint8_t Turn_Angle(float Angle);
uint8_t runWithAngle(float angle_want, float speed);
uint8_t Stage_turn_Angle(float Angle);
void Turn_Angle_Relative(float Angle1);
void Turn_Angle_Relative_Open(float Angle1);
void mpuZreset(float sensorangle, float referangle);
void Turn_Angle360(void);
void FreeTurn(float Angle, float L, float R);

#define get_pitch() imu.pitch

extern uint8_t gryo_turn;
extern volatile struct Angle angle;
extern uint8_t Turn360_Flag;

#endif
