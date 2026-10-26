#ifndef DR16_H
#define DR16_H

#include <stdint.h>

#define DR16_FRAME_LENGTH   18U
#define DR16_RX_BUF_NUM     36U
#define DR16_CH_COUNT       5U

#define DR16_CH_VALUE_MIN   364
#define DR16_CH_VALUE_MID   1024
#define DR16_CH_VALUE_MAX   1684
#define DR16_CH_RANGE       660
#define DR16_DEAD_ZONE      20

#define DR16_SW_UP          1U
#define DR16_SW_MID         3U
#define DR16_SW_DOWN        2U

#define DR16_TIMEOUT_MS     100U
#define DR16_RESTART_MS     500U

#define DR16_KEY_W          ((uint16_t)1U << 0)
#define DR16_KEY_S          ((uint16_t)1U << 1)
#define DR16_KEY_A          ((uint16_t)1U << 2)
#define DR16_KEY_D          ((uint16_t)1U << 3)
#define DR16_KEY_SHIFT      ((uint16_t)1U << 4)
#define DR16_KEY_CTRL       ((uint16_t)1U << 5)
#define DR16_KEY_Q          ((uint16_t)1U << 6)
#define DR16_KEY_E          ((uint16_t)1U << 7)
#define DR16_KEY_R          ((uint16_t)1U << 8)
#define DR16_KEY_F          ((uint16_t)1U << 9)
#define DR16_KEY_G          ((uint16_t)1U << 10)
#define DR16_KEY_Z          ((uint16_t)1U << 11)
#define DR16_KEY_X          ((uint16_t)1U << 12)
#define DR16_KEY_C          ((uint16_t)1U << 13)
#define DR16_KEY_V          ((uint16_t)1U << 14)
#define DR16_KEY_B          ((uint16_t)1U << 15)

typedef struct
{
    int16_t ch[DR16_CH_COUNT];
    uint8_t s[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_press_l;
    uint8_t mouse_press_r;
    uint16_t key;
} dr16_ctrl_t;

void Dr16_Init(void);
void Dr16_Irq_Handler(void);
void Dr16_Task(void);

const dr16_ctrl_t *Dr16_Get(void);
uint8_t Dr16_Is_Online(void);
int16_t Dr16_Get_Channel_Milli(uint8_t index);
int16_t Dr16_Get_Channel_Milli_Scaled(uint8_t index, int16_t full_milli);
int16_t Dr16_Get_Channel_Centi(uint8_t index);
float Dr16_Get_Channel_Norm(uint8_t index);
uint8_t Dr16_Get_Switch(uint8_t index);

#endif
