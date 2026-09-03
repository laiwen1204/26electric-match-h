#ifndef __CAR_H
#define __CAR_H

#include "ti_msp_dl_config.h"

/* DRV8701 方向控制引脚 (PHASE) */
#define PH_L_OUT(X)  ((X) ? DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14)   : DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14))
#define PH_R_OUT(X)  ((X) ? DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)   : DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12))

void Motor_Init(void);
void Motor_L_Run_forward(uint16_t speed);
void Motor_L_Run_back(uint16_t speed);
void Motor_L_Stop(void);
void Motor_R_Run_forward(uint16_t speed);
void Motor_R_Run_back(uint16_t speed);
void Motor_R_Stop(void);
void Motor_All_Stop(void);
void car_forward(uint16_t speed);
void car_back(uint16_t speed);
void car_turnleft(uint16_t speed);
void car_turnright(uint16_t speed);

#endif
