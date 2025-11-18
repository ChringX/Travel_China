#ifndef __TURN_H
#define __TURN_H

#include "sys.h"
#include "imu_task.h"

struct Angle {
	float AngleT;
	float AngleG;
};

extern volatile struct Angle angle;
float need2turn(float nowangle,float targetangle);
void mpuZreset(float sensorangle ,float referangle);
float getAngleZ(void);
	
uint8_t Turn_Angle(float Angle);
void Turn_Angle_Relative(float Angle1);
void Turn_Angle_Relative_Open(float Angle1);
uint8_t runWithAngle(float angle_want,float speed);
uint8_t Drift(float speed,float Angle);
void AdCircle(float speed, float radius);
uint8_t Stage_turn_Angle(float Angle);	
uint8_t Turn_Fix(float Angle);
float Get_F(int Cspeed);
static inline float get_pitch(void)
{
	return imu.pitch;
}
void Change_MODE(void);
void Turn_Angle360(void);
void Fix_Stage(float angle);
void FreeTurn(float Angle,float L,float R);
float Modified_Angle(float angle,float angle_add);
extern uint8_t gryo_turn;
#endif
