#ifndef __imu_h__
#define __imu_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
struct Imu
{
	float yaw;
	float roll;
	float pitch;

	float compensateZ;
	float compensatePitch;
};
extern struct Imu imu;
extern float basic_p;
extern float basic_y;
void gyro_init(uint32_t bound);
void imu_receive_init(void);
#endif
