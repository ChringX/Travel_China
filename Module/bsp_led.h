#ifndef BSP_LED_H
#define BSP_LED_H

#include "sys.h"

//翻转LED C0电平
#define LED_C0_Toggle()   do{HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0); } while(0)
//翻转LED C1电平
#define LED_C1_Toggle()   do{HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1); } while(0)

#endif
