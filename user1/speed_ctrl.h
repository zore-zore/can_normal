#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include <stdint.h>

void Speed_Ctrl_Init(void);
void Speed_Ctrl_Reset(void);
void Speed_Ctrl_Update(const int32_t mix[4]);

#endif
