#include "IBM_gyro_correct.h"
#include "main.h"
#include "BMI088driver.h"
#include "dr16.h"

#define GYRO_CORRECT_Z_DIR          (1.0f)
#define GYRO_CORRECT_SAMPLE_MS      5U      //陀螺仪采样周期，单位ms
#define GYRO_CORRECT_BIAS_SAMPLES   230U   //陀螺仪采样230次，用于基准点
#define GYRO_CORRECT_BIAS_SKIP      70U    //陀螺仪采样前70次数据不要
#define GYRO_CORRECT_LPF_ALPHA      0.611f
#define GYRO_CORRECT_RATE_MAX       20.0f
#define GYRO_CORRECT_FORWARD_MIN    100
#define GYRO_CORRECT_DT_S           0.001f
#define GYRO_CORRECT_KP             300.0f
#define GYRO_CORRECT_KI             600.0f
#define GYRO_CORRECT_CORR_MAX       200.0f
#define GYRO_CORRECT_IOUT_MAX       120.0f
#define GYRO_CORRECT_RAMP_STEP      0.005f

static volatile float gyro_correct_rate;
static float gyro_correct_bias;
static float gyro_correct_lpf;
static float gyro_correct_integ;
static float gyro_correct_ramp;
static uint8_t gyro_correct_ready;
static uint8_t gyro_correct_run;

static float gyro_correct_read(void)  //BMI088 刚读出来的三轴角速度
{
    fp32 gyro[3];
    fp32 accel[3];
    fp32 temperature;

    BMI088_read(gyro, accel, &temperature);
    gyro_correct_lpf += GYRO_CORRECT_LPF_ALPHA * (gyro[2] - gyro_correct_lpf);

    return gyro_correct_lpf;
}

void Gyro_Correct_Init(void)  //陀螺仪自转修正初始化，基准点校正
{
    float sum = 0.0f;
    uint16_t i;

    gyro_correct_ready = 0U;
    gyro_correct_run = 0U;
    gyro_correct_bias = 0.0f;
    gyro_correct_lpf = 0.0f;
    gyro_correct_integ = 0.0f;
    gyro_correct_ramp = 0.0f;
    gyro_correct_rate = 0.0f;

    if (BMI088_init() != BMI088_NO_ERROR)
    {
        return;
    }

    for (i = 0U; i < GYRO_CORRECT_BIAS_SAMPLES; i++)   //采样次数的计数器
    {
        HAL_Delay(GYRO_CORRECT_SAMPLE_MS);
        if  (i<GYRO_CORRECT_BIAS_SKIP )
        {
            (void)gyro_correct_read();   //前70次采集不计入平均值，避免陀螺仪刚启动时的漂移
        }
        else
        {
            sum += gyro_correct_read();   //每次采集相加
        }
    }

    gyro_correct_bias = sum / (float)(GYRO_CORRECT_BIAS_SAMPLES - GYRO_CORRECT_BIAS_SKIP);
    gyro_correct_ready = 1U;
}

void Gyro_Correct_Task(void)
{
    static uint32_t gyro_correct_last_tick;
    uint32_t now;
    float rate;

    if (gyro_correct_ready == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if ((uint32_t)(now - gyro_correct_last_tick) < GYRO_CORRECT_SAMPLE_MS)
    {
        return;
    }

    gyro_correct_last_tick = now;

    rate = gyro_correct_read() - gyro_correct_bias;

    if (rate > GYRO_CORRECT_RATE_MAX)
    {
        rate = GYRO_CORRECT_RATE_MAX;
    }
    else if (rate < -GYRO_CORRECT_RATE_MAX)
    {
        rate = -GYRO_CORRECT_RATE_MAX;
    }

    gyro_correct_rate = rate * GYRO_CORRECT_Z_DIR;
}

int32_t Gyro_Correct_Update(uint8_t stick_right_active, int32_t forward, int32_t rotate)
{
    float rate;
    float err;
    float corr;
    uint8_t want_run;

    if (gyro_correct_ready == 0U)
    {
        return 0;
    }

    want_run = 0U;

    if ((stick_right_active != 0U)   //rotate自转容器，forward向前容器，右杆有动作时不响应左杆
        && (rotate == 0)    
        && ((forward >= GYRO_CORRECT_FORWARD_MIN) || (forward <= -GYRO_CORRECT_FORWARD_MIN))
        && (Dr16_Is_Online() != 0U))
    {
        want_run = 1U;
    }

    if (want_run != 0U)
    {
        gyro_correct_run = 1U;

        if (gyro_correct_ramp < 1.0f)
        {
            gyro_correct_ramp += GYRO_CORRECT_RAMP_STEP;

            if (gyro_correct_ramp > 1.0f)
            {
                gyro_correct_ramp = 1.0f;
            }
        }
    }
    else if (gyro_correct_run != 0U)
    {
        gyro_correct_ramp -= GYRO_CORRECT_RAMP_STEP;

        if (gyro_correct_ramp <= 0.0f)
        {
            gyro_correct_ramp = 0.0f;
            gyro_correct_run = 0U;
        }
    }

    if (gyro_correct_ramp <= 0.0f)   //PI 计算
    {
        return 0;
    }

    rate = gyro_correct_rate;
    err = -rate;

    gyro_correct_integ += GYRO_CORRECT_KI * err * GYRO_CORRECT_DT_S;

    if (gyro_correct_integ > GYRO_CORRECT_IOUT_MAX)
    {
        gyro_correct_integ = GYRO_CORRECT_IOUT_MAX;
    }
    else if (gyro_correct_integ < -GYRO_CORRECT_IOUT_MAX)
    {
        gyro_correct_integ = -GYRO_CORRECT_IOUT_MAX;
    }

    corr = GYRO_CORRECT_KP * err + gyro_correct_integ;

    if (corr > GYRO_CORRECT_CORR_MAX)
    {
        corr = GYRO_CORRECT_CORR_MAX;
    }
    else if (corr < -GYRO_CORRECT_CORR_MAX)
    {
        corr = -GYRO_CORRECT_CORR_MAX;
    }

    return (int32_t)(corr * gyro_correct_ramp);
}
