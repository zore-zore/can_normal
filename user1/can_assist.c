#include "can_assist.h"
#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "dr16.h"
#include "speed_ctrl.h"

#define CAN_ASSIST_CH_FORWARD   3U   //左摇杆前后
#define CAN_ASSIST_CH_STRAFE    2U   //左摇杆左右
#define CAN_ASSIST_CH_FORWARD_R 1U   //右摇杆前后
#define CAN_ASSIST_CH_ROTATE    0U   //右摇杆左右
#define CAN_ASSIST_CURRENT_MAX  1000  //转速满量程
#define CAN_ASSIST_PERIOD_MS    1U  

static const int8_t can_assist_dir[4] = {1, 1, -1, -1};   //逆运动学方向系数，顺序为前左、前右、后左、后右
static const int8_t can_assist_strafe_dir[4] = {1, -1, -1, 1};   //逆运动学方向系数，顺序为前左、前右、后左、后右
static const int8_t can_assist_rotate_dir[4] = {1, 1, 1, 1};   //逆运动学方向系数，顺序为前左、前右、后左、后右
static uint32_t can_assist_last_tick;
static volatile uint8_t can_assist_enabled;
static uint8_t can_assist_was_online;

void Can_Assist_Init(void)
{
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

    forward = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_FORWARD) //通过遥控器比例，调转速大小
                        * (float)CAN_ASSIST_CURRENT_MAX);
    strafe = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_STRAFE)
                       * (float)CAN_ASSIST_CURRENT_MAX);
    rotate = 0;

    if ((forward == 0) && (strafe == 0))  //左侧优先，左杆有动作时不响应右杆
    {
        forward = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_FORWARD_R)
                            * (float)CAN_ASSIST_CURRENT_MAX);
        rotate = (int32_t)(Dr16_Get_Channel_Norm(CAN_ASSIST_CH_ROTATE)
                           * (float)CAN_ASSIST_CURRENT_MAX);
    }

    for (i = 0U; i < 4U; i++)  //逆运动学
    {
        mix[i] = forward * can_assist_dir[i]   //逆运动学计算
                 + strafe * can_assist_strafe_dir[i]    //逆运动学计算
                 + rotate * can_assist_rotate_dir[i];    //逆运动学计算

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
