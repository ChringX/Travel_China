#include "main_task.h"
#include "rudder_control.h"
#include "uart.h"
#include "imu.h"
#include "uart.h"
#include "turn.h"
#include "map.h"
#include "barrier.h"
#include "bsp_buzzer.h"
#include "bsp_linefollower.h"
#include "scaner.h"
#include "speed_ctrl.h"
#include "encoder.h"
#include "barrier.h"
#include "motor_task.h"
#include "openmv.h"
#include "math.h"
#include "barrier.h"
#include "sin_generate.h"
#include "gray.h"
#include "QR.h"



/*主任务*/
void main_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //获取系统节拍、

	zhunbei(); // 启动流程
	
	encoder_clear(); // 路程记录清零
	Motor_Control(is_Line, SPEED0, SPEED0, 0);

	while (1)
	{
		/*二轮处理*/
		if(map.routetime == 1)
		{
			map.routetime = 2;
			get_newroute();

			float mpuZ_reset_val;
			for (uint8_t i = 0; i < 10; i++)
			{
				delay_ms(20);
				mpuZ_reset_val += imu.yaw;
				basic_p += imu.pitch;
			}
			mpuZ_reset_val /= 10;
			basic_p /= 10;
			basic_y = mpuZ_reset_val;
			mpuZreset(mpuZ_reset_val, nodesr.nowNode.angle); // 把此时角度变为此结点角度
			zhunbei();

			encoder_clear(); // 路程记录清零
			Motor_Control(is_Line, SPEED0, SPEED0, 0);
		}
		
		/*节点间处理*/
		Cross();
		
		/*二轮结束处理*/
		if(map.routetime==3)
			CarBrake_Stop();
	
		vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
	}
}
