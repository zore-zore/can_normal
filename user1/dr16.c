#include "dr16.h"
#include "usart.h"
#include <stddef.h>

extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t dr16_rx_buf[2][DR16_RX_BUF_NUM];
static dr16_ctrl_t dr16_ctrl[2];
static volatile uint8_t dr16_ctrl_index;
static volatile uint32_t dr16_frame_count;
static uint32_t dr16_last_frame_count;
static uint32_t dr16_last_tick;
static volatile uint8_t dr16_online;

static void dr16_parse(const uint8_t *buf, dr16_ctrl_t *ctrl)
{
    ctrl->ch[0] = (int16_t)((int16_t)((buf[0] | (buf[1] << 8)) & 0x07FF) - DR16_CH_VALUE_MID);
    ctrl->ch[1] = (int16_t)((int16_t)(((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - DR16_CH_VALUE_MID);
    ctrl->ch[2] = (int16_t)((int16_t)(((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - DR16_CH_VALUE_MID);
    ctrl->ch[3] = (int16_t)((int16_t)(((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - DR16_CH_VALUE_MID);

    ctrl->s[0] = (uint8_t)((buf[5] >> 4) & 0x0003);
    ctrl->s[1] = (uint8_t)(((buf[5] >> 4) & 0x000C) >> 2);

    ctrl->mouse_x = (int16_t)(buf[6] | (buf[7] << 8));
    ctrl->mouse_y = (int16_t)(buf[8] | (buf[9] << 8));
    ctrl->mouse_z = (int16_t)(buf[10] | (buf[11] << 8));
    ctrl->mouse_press_l = buf[12];
    ctrl->mouse_press_r = buf[13];
    ctrl->key = (uint16_t)(buf[14] | (buf[15] << 8));
    ctrl->ch[4] = (int16_t)((int16_t)(buf[16] | (buf[17] << 8)) - DR16_CH_VALUE_MID);
}

void Dr16_Init(void)
{
    HAL_NVIC_DisableIRQ(USART3_IRQn);

    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t)&USART3->DR;
    hdma_usart3_rx.Instance->M0AR = (uint32_t)dr16_rx_buf[0];
    hdma_usart3_rx.Instance->M1AR = (uint32_t)dr16_rx_buf[1];
    hdma_usart3_rx.Instance->NDTR = DR16_RX_BUF_NUM;
    hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    dr16_last_frame_count = dr16_frame_count;
    dr16_last_tick = HAL_GetTick();

    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void Dr16_Irq_Handler(void)
{
    uint32_t length;

    if ((huart3.Instance->SR & UART_FLAG_RXNE) != 0U)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        return;
    }

    if ((huart3.Instance->SR & UART_FLAG_IDLE) == 0U)
    {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(&huart3);

    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    length = DR16_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
    hdma_usart3_rx.Instance->NDTR = DR16_RX_BUF_NUM;

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == 0U)
    {
        SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
        __HAL_DMA_ENABLE(&hdma_usart3_rx);

        if (length == DR16_FRAME_LENGTH)
        {
            dr16_parse((const uint8_t *)dr16_rx_buf[0], &dr16_ctrl[dr16_ctrl_index ^ 1U]);
            dr16_ctrl_index = (uint8_t)(dr16_ctrl_index ^ 1U);
            dr16_frame_count++;
        }
    }
    else
    {
        CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
        __HAL_DMA_ENABLE(&hdma_usart3_rx);

        if (length == DR16_FRAME_LENGTH)
        {
            dr16_parse((const uint8_t *)dr16_rx_buf[1], &dr16_ctrl[dr16_ctrl_index ^ 1U]);
            dr16_ctrl_index = (uint8_t)(dr16_ctrl_index ^ 1U);
            dr16_frame_count++;
        }
    }
}

void Dr16_Task(void)
{
    uint32_t now;
    uint32_t count;

    now = HAL_GetTick();
    count = dr16_frame_count;

    if (count != dr16_last_frame_count)
    {
        dr16_last_frame_count = count;
        dr16_last_tick = now;
        dr16_online = 1U;
        return;
    }

    if ((now - dr16_last_tick) >= DR16_TIMEOUT_MS)
    {
        dr16_online = 0U;

        if ((now - dr16_last_tick) >= DR16_RESTART_MS)
        {
            Dr16_Init();
        }
    }
}

const dr16_ctrl_t *Dr16_Get(void)
{
    return &dr16_ctrl[dr16_ctrl_index];
}

uint8_t Dr16_Is_Online(void)
{
    return dr16_online;
}

static int32_t dr16_normalize(int16_t ch, int32_t full_milli)
{
    int32_t milli;

    if ((ch > -DR16_DEAD_ZONE) && (ch < DR16_DEAD_ZONE))
    {
        return 0;
    }

    milli = ((int32_t)ch * full_milli) / DR16_CH_RANGE;

    if (milli > full_milli)
    {
        milli = full_milli;
    }
    else if (milli < -full_milli)
    {
        milli = -full_milli;
    }

    return milli;
}

int16_t Dr16_Get_Channel_Milli_Scaled(uint8_t index, int16_t full_milli)
{
    const dr16_ctrl_t *ctrl;

    if (index >= DR16_CH_COUNT)
    {
        return 0;
    }

    ctrl = Dr16_Get();

    return (int16_t)dr16_normalize(ctrl->ch[index], full_milli);
}

int16_t Dr16_Get_Channel_Milli(uint8_t index)
{
    return Dr16_Get_Channel_Milli_Scaled(index, 1000);
}

/* 归一化：轴值除以 660，保留两位小数，第三位四舍五入；小于 0.5% 自然归零 */

static int32_t dr16_normalize_centi(int16_t ch)
{
    int32_t magnitude;
    int32_t centi;

    magnitude = (ch < 0) ? -(int32_t)ch : (int32_t)ch;
    centi = (magnitude * 100 + (DR16_CH_RANGE / 2)) / DR16_CH_RANGE;

    if (centi > 100)
    {
        centi = 100;
    }
    if (centi < 4)
    {
        centi = 0;
    } 


    return (ch < 0) ? -centi : centi;
}

int16_t Dr16_Get_Channel_Centi(uint8_t index)
{
    if (index >= DR16_CH_COUNT)
    {
        return 0;
    }

    return (int16_t)dr16_normalize_centi(Dr16_Get()->ch[index]);
}

float Dr16_Get_Channel_Norm(uint8_t index)
{
    return (float)Dr16_Get_Channel_Centi(index) / 100.0f;
}

uint8_t Dr16_Get_Switch(uint8_t index)
{
    if (index > 1U)
    {
        return 0U;
    }

    return Dr16_Get()->s[index];
}
