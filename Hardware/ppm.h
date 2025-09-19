#ifndef __PPM_H
#define __PPM_H
#include "stm32f10x.h"                  // Device header

void PPM_Input_Init(void);
void ppm_update(void);
void ppm_send(void);



#endif
