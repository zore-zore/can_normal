#ifndef UART_ASSIST_H
#define UART_ASSIST_H

#include <stdint.h>

void Uart_Assist_Init(void);
void Uart_Assist_Task(void);
void Uart_Assist_Send(const char *text);
void Uart_Assist_SetPaused(uint8_t paused);
uint8_t Uart_Assist_IsPaused(void);

#endif
