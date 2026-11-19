#include "Moto.h"
#include "Gimbal.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "usart.h"

uint32_t last_gimbal_ctrl_time = 0;

void Task_MotorCallback(void)
{
    /* 底盘任务独占 USART1，云台继续沿用 USART2。 */
    last_gimbal_ctrl_time++;
		if(last_gimbal_ctrl_time >= 20)
    {
        last_gimbal_ctrl_time = 0;
        set_gimbal_ctrl(&huart2, gimbal.yawMotor.targetPulse, gimbal.liftMotor.targetPulse);
    }
}

void OS_MotorCallback(void* argument)
{
    for(;;)
    {
        Task_MotorCallback();
        osDelay(1);
    }
}
