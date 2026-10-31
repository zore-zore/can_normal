#include "can_assist.h"
#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "dr16.h"
#include "speed_ctrl.h"
#include "IBM_gyro_correct.h"

#define CAN_ASSIST_CH_FORWARD   3U   //左摇杆前后
#define CAN_ASSIST_CH_STRAFE    2U   //左摇杆左右
#define CAN_ASSIST_CH_FORWARD_R 1U   //右摇杆前后
#define CAN_ASSIST_CH_ROTATE    0U   //右摇杆左右
#define CAN_ASSIST_CURRENT_MAX  1000  //转速满量程
#define CAN_ASSIST_PERIOD_MS    1U  

static const int8_t can_assist_dir[4] = {1, 1, -1, -1};   //顺序为左上、左下、右下、右上。电调id：左上1，左下2，右上4，右下3。
static const int8_t can_assist_strafe_dir[4] = {1, -1, -1, 1};   //逆运动学方向系数，顺序为前左、前右、后左、后右
static const int8_t can_assist_rotate_dir[4] = {1, 1, 1, 1};   //逆运动学方向系数，顺序为前左、前右、后左、后右
static uint32_t can_assist_last_tick;
static volatile uint8_t can_assist_enabled;
static uint8_t can_assist_was_online;

void Can_Assist_Init(void)    
{
    Gyro_Correct_Init();
    can_filter_init();
    Speed_Ctrl_Init();
    can_assist_last_tick = HAL_GetTick() - CAN_ASSIST_PERIOD_MS;
    can_assist_enabled = 1U;
    can_assist_was_online = 0U;
}

void Can_Assist_Control(void)
{
    int32_t forward;
    int32_t strafe;
    int32_t rotate;
    int32_t mix[4];
    int32_t corr;
    uint8_t right_active;
    uint8_t i;

    if (can_assist_enabled == 0U)
    {
        return;
    }

    if (Dr16_Is_Online() == 0U)
    {
        if (can_assist_was_online != 0U)
        {
            can_assist_was_online = 0U;
            Speed_Ctrl_Reset();
        }

        mix[0] = 0;
        mix[1] = 0;
        mix[2] = 0;
        mix[3] = 0;
        Speed_Ctrl_Update(mix);
        return;
    }

    can_assist_was_online = 1U;

    forward = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_FORWARD) //左杆前后
                        * (float)CAN_ASSIST_CURRENT_MAX);
    strafe = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_STRAFE)  //左杆左右
                       * (float)CAN_ASSIST_CURRENT_MAX);
    rotate = 0;   //初始化
    right_active = 0U;  
    
    //forward,向前的容器。strafe,向右的容器。rotate,自转的容器。

    if ((forward == 0) && (strafe == 0))  //左侧优先，左杆有动作时不响应右杆
    {
        forward = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_FORWARD_R)
                            * (float)CAN_ASSIST_CURRENT_MAX);
        rotate = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_ROTATE)
                           * (float)CAN_ASSIST_CURRENT_MAX);
        if  (rotate <= (float)CAN_ASSIST_CURRENT_MAX * 0.5F && rotate >= -(float)CAN_ASSIST_CURRENT_MAX * 0.5F) 
         //右杆自转死区,实现右杆前后优先
        {
            rotate = 0;
        }
        else
        {
            rotate = rotate / 2; //右杆自转量减半，避免过快，不能避免突变
        }
        right_active = 1U;

    }

    
    corr = Gyro_Correct_Update(right_active, forward, rotate);

    for (i = 0U; i < 4U; i++)  //逆运动学
    {
        mix[i] = forward * can_assist_dir[i]   //逆运动学计算
                 + strafe * can_assist_strafe_dir[i]    //逆运动学计算
                 + rotate * can_assist_rotate_dir[i];    //逆运动学计算

        mix[i] += corr;

        if (mix[i] > CAN_ASSIST_CURRENT_MAX) // 保护程序
        {
            mix[i] = CAN_ASSIST_CURRENT_MAX;
        }
        else if (mix[i] < -CAN_ASSIST_CURRENT_MAX)
        {
            mix[i] = -CAN_ASSIST_CURRENT_MAX;
        }
    }

    Speed_Ctrl_Update(mix);
}

void Can_Assist_Task(void)
{
    uint32_t now;

    Gyro_Correct_Task();

    now = HAL_GetTick();

    if ((uint32_t)(now - can_assist_last_tick) < CAN_ASSIST_PERIOD_MS)
    {
        return;
    }

    can_assist_last_tick = now;

    if ((CAN1->ESR & CAN_ESR_BOFF) != 0U)
    {
        can_assist_enabled = 0U;
        (void)HAL_CAN_Init(&hcan1);
        can_filter_init();
        can_assist_enabled = 1U;
    }
}
