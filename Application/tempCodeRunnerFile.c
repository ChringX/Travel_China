
		vTaskDelay(500);

		/*判断开哪边的MV*/
		Color_Right = Color_Left = 0;
		if (flag <= 10) // 去
			Open_MV_R();
		else
			Open_MV_L();
		
		/*等看完灯*/
		uint16_t outtime = 0;
		while (Color_Right == 0 && Color_Left == 0)
		{
			outtime++;
			if (outtime >= 750)
				break;
			vTaskDelay(2);
		}