#include "usdelay.h"
#include "main.h"
#include "tim.h"
//extern TIM_HandleTypeDef htim1; //外边已经定义好了结构体，直接引用，不是重新创建的

void delay_micro_sec(uint8_t us)
{
    HAL_TIM_Base_Start(&htim1);
    while(__HAL_TIM_GET_COUNTER(&htim1)<us);
    HAL_TIM_Base_Stop(&htim1);
}