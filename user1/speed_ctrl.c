#include "speed_ctrl.h"
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"

#define SPEED_CTRL_MAX_RPM      1000  //摇杆推到底时的目标转子转速，单位：转/分
#define SPEED_CTRL_CURRENT_MAX  1000  //PID 输出的电流上限，16384 对应 20A，单位：mA
#define SPEED_CTRL_IOUT_MAX     500

static const fp32 speed_ctrl_pid_param[3] = {10.0f, 0.2f, 0.0f};
/* 电机的反馈方向，正常情况四个都填 1，不要拿它来纠轮子的正反转。
   含义是「正电流下去时这个电机 speed_rpm 报出来的符号」。它只跟电机的相线接法
   有关：正装、反装都不影响 —— 反装只是让轮子往反方向推，speed_rpm 照样是正的，
   那种情况要去改 can_assist_dir。
   只有某台电机换过三相线或者编码器线序、反馈本身是反的，才把它单独填 -1。填反
   了负反馈就变成正反馈，闭环发散，表现为电机乱转、抖振，摇杆回中也压不住。
   台架验证：把车架空，只推一点点前进，四个轮子同向平滑转动就是对的；乱转就是
   这一位填反了。 */
static const int8_t speed_ctrl_sign[4] = {1, 1, 1, 1};
static pid_type_def speed_ctrl_pid[4];

void Speed_Ctrl_Init(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        PID_init(&speed_ctrl_pid[i], PID_POSITION, speed_ctrl_pid_param,
                 (fp32)SPEED_CTRL_CURRENT_MAX, (fp32)SPEED_CTRL_IOUT_MAX);
    }
}

void Speed_Ctrl_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        PID_clear(&speed_ctrl_pid[i]);
    }
}

void Speed_Ctrl_Update(const int32_t mix[4])
{
    const motor_measure_t *measure;
    int32_t target_rpm;
    int16_t current[4];
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        measure = get_chassis_motor_measure_point(i);

        target_rpm = mix[i] * SPEED_CTRL_MAX_RPM / 1000;

        if (target_rpm > SPEED_CTRL_MAX_RPM)
        {
            target_rpm = SPEED_CTRL_MAX_RPM;
        }
        else if (target_rpm < -SPEED_CTRL_MAX_RPM)
        {
            target_rpm = -SPEED_CTRL_MAX_RPM;
        }

        current[i] = (int16_t)PID_calc(&speed_ctrl_pid[i],
                                       (fp32)(measure->speed_rpm * speed_ctrl_sign[i]),
                                       (fp32)target_rpm);
    }

    CAN_cmd_chassis(current[0], current[1], current[2], current[3]);
}
