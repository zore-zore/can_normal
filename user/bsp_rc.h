#ifndef BSP_RC_H    //遥控器底层驱动之一，暂时不用，user1里dr16代替
#define BSP_RC_H
#include "struct_typedef.h"

extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
