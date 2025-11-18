#ifndef __imu_task_h__
#define __imu_task_h__
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
void gyro_init(uint32_t bound);
void imu_receive_init(void);
#endif
