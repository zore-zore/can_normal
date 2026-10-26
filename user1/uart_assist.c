#include "uart_assist.h"
#include "usart.h"
#include "dr16.h"
#include <math.h>
#include <stddef.h>

#define ASSIST_SINE_PERIOD_MS   2000U   /* 正弦波走完一个完整周期的时间，单位 ms */
#define ASSIST_SINE_SAMPLE_MS   10U     /* 每隔多少 ms 发送一个采样点，越大越慢 */
#define ASSIST_SINE_AMPLITUDE   1000
#define ASSIST_PI               3.14159265f
#define ASSIST_TEXT_SIZE        24U

#define ASSIST_STREAM_OFF       0U
#define ASSIST_STREAM_SINE      1U
#define ASSIST_STREAM_STICK     2U

#define ASSIST_STICK_CH_VERT    2U
#define ASSIST_STICK_CH_HORZ    3U
#define ASSIST_STICK_VERT_MILLI 1000   //19-20，显示serialplot的纵轴范围
#define ASSIST_STICK_HORZ_MILLI 1000

static uint8_t assist_rx_byte;
static volatile uint8_t assist_cmd;
static volatile uint8_t assist_stream;
static volatile uint8_t assist_paused;
static uint32_t assist_last_sample_tick;

static void assist_send(const char *text, uint16_t length)
{
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)text, length, 100U);
}

static void assist_send_milli(const int32_t *values, uint8_t count)
{
    char text[ASSIST_TEXT_SIZE];
    char *p;
    uint32_t magnitude;
    int32_t value;
    uint8_t i;
    uint8_t digit;

    p = &text[ASSIST_TEXT_SIZE - 1U];
    *p = '\n';

    for (i = count; i > 0U; i--)
    {
        value = values[i - 1U];

        if (value < -9999)
        {
            value = -9999;
        }
        else if (value > 9999)
        {
            value = 9999;
        }

        magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;

        for (digit = 0U; digit < 3U; digit++)
        {
            *--p = (char)('0' + (magnitude % 10U));
            magnitude /= 10U;
        }

        *--p = '.';

        do
        {
            *--p = (char)('0' + (magnitude % 10U));
            magnitude /= 10U;
        }
        while (magnitude > 0U);

        if (value < 0)
        {
            *--p = '-';
        }

        if (i > 1U)
        {
            *--p = ',';
        }
    }

    assist_send(p, (uint16_t)(&text[ASSIST_TEXT_SIZE - 1U] - p) + 1U);
}

static void assist_send_sine(void)
{
    int32_t value;

    value = (int32_t)(sinf(2.0f * ASSIST_PI
                           * (float)(HAL_GetTick() % ASSIST_SINE_PERIOD_MS)
                           / (float)ASSIST_SINE_PERIOD_MS)
                      * (float)ASSIST_SINE_AMPLITUDE);

    assist_send_milli(&value, 1U);
}

void Uart_Assist_Init(void)
{
    assist_cmd = 0U;
    assist_stream = ASSIST_STREAM_OFF;
    assist_paused = 0U;
    assist_last_sample_tick = HAL_GetTick();
    (void)HAL_UART_Receive_IT(&huart6, &assist_rx_byte, 1U);
}

void Uart_Assist_Task(void)
{
    uint8_t cmd;
    uint32_t now;
    int32_t columns[2];

    cmd = assist_cmd;
//uart6反馈代码
    if (cmd != 0U)
    {
        assist_cmd = 0U;

        if (cmd == (uint8_t)'0')
        {
            assist_stream = ASSIST_STREAM_OFF;
            assist_paused = 0U;
            assist_send("hello\r\n", 7U);
        }
        else if (cmd == (uint8_t)'1')
        {
            assist_paused = 0U;
            assist_stream = ASSIST_STREAM_SINE;
            assist_last_sample_tick = HAL_GetTick() - ASSIST_SINE_SAMPLE_MS;
        }
        else if (cmd == (uint8_t)'2')
        {
            assist_stream = ASSIST_STREAM_OFF;
        }
        else if (cmd == (uint8_t)'3')
        {
            assist_paused = 0U;
            assist_stream = ASSIST_STREAM_STICK;
            assist_last_sample_tick = HAL_GetTick() - ASSIST_SINE_SAMPLE_MS;
        }
    }

    if ((assist_stream == ASSIST_STREAM_OFF) || (assist_paused != 0U))
    {
        return;
    }

    now = HAL_GetTick();

    if ((uint32_t)(now - assist_last_sample_tick) < ASSIST_SINE_SAMPLE_MS)
    {
        return;
    }

    assist_last_sample_tick = now;

    if (assist_stream == ASSIST_STREAM_SINE)
    {
        assist_send_sine();
    }
    else  //状态[3]下，uart6发送遥控器数据
    {
        if (Dr16_Is_Online() != 0U)
        {
            columns[0] = (int16_t)(Dr16_Get_Channel_Centi(ASSIST_STICK_CH_VERT)
                                   * (ASSIST_STICK_VERT_MILLI / 100));
            columns[1] = (int16_t)(Dr16_Get_Channel_Centi(ASSIST_STICK_CH_HORZ)
                                   * (ASSIST_STICK_HORZ_MILLI / 100));
        }
        else
        {
            columns[0] = 0;
            columns[1] = 0;
        }

        assist_send_milli(columns, 2U);
    }
}

void Uart_Assist_SetPaused(uint8_t paused)
{
    assist_paused = (paused != 0U) ? 1U : 0U;
}

uint8_t Uart_Assist_IsPaused(void)
{
    return assist_paused;
}

void Uart_Assist_Send(const char *text)
{
    uint16_t length;

    if ((text == NULL) || (assist_paused != 0U))
    {
        return;
    }

    length = 0U;

    while (text[length] != '\0')
    {
        length++;
    }

    if (length != 0U)
    {
        assist_send(text, length);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6)
    {
        return;
    }

    assist_cmd = assist_rx_byte;
    (void)HAL_UART_Receive_IT(&huart6, &assist_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        (void)HAL_UART_Receive_IT(&huart6, &assist_rx_byte, 1U);
    }
}
