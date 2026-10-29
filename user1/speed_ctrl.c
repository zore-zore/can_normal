#include "speed_ctrl.h"
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"

#define SPEED_CTRL_MAX_RPM      1000  //摇杆推到底时的目标转子转速，单位：转/分。因素之一
#define SPEED_CTRL_CURRENT_MAX  1000  //PID 输出的电流上限，16384 对应 20A，单位：mA。因素之一
#define SPEED_CTRL_IOUT_MAX     500   //积分项上限，防止积分饱和，约 0.61A
#define SPEED_CTRL_DT_S         0.001f

static const fp32 speed_ctrl_pid_param[3] = {10.0f, 200.0f, 0.005f};  
//PID 参数，顺序为 Kp、Ki、Kd。Ki 的单位是 1/秒，Kd 的单位是秒，初始化时按采样周期换算
static const int8_t speed_ctrl_sign[4] = {1, 1, 1, 1};
static pid_type_def speed_ctrl_pid[4];

void Speed_Ctrl_Init(void) 
{
    fp32 param[3];
    uint8_t i;

    param[0] = speed_ctrl_pid_param[0];
    param[1] = speed_ctrl_pid_param[1] * SPEED_CTRL_DT_S;
    param[2] = speed_ctrl_pid_param[2] / SPEED_CTRL_DT_S;

    for (i = 0U; i < 4U; i++)
    {
        PID_init(&speed_ctrl_pid[i], PID_POSITION, param,
                 (fp32)SPEED_CTRL_CURRENT_MAX, (fp32)SPEED_CTRL_IOUT_MAX);
    }
}

void Speed_Ctrl_Reset(void)  //初始化
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
