#ifndef IBM_GYRO_CORRECT_H
#define IBM_GYRO_CORRECT_H

#include <stdint.h>

void Gyro_Correct_Init(void);
void Gyro_Correct_Task(void);
int32_t Gyro_Correct_Update(uint8_t stick_right_active, int32_t forward, int32_t rotate);

#endif
