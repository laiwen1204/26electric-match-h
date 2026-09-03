#ifndef __VOFA_H__
#define __VOFA_H__

#include "app.h"     /* 车辆状态全局变量: CarMode/CarCommand/BaseSpeed/TrackSpeed/各PID */
#include "usart.h"   /* vofa_rx_packet / vofa_rx_flag (定义在usart.c, 与RX中断耦合) */

/* VOFA指令解析：slider调参 + button运动控制, 主循环每轮调用一次 */
void ParseVOFACommand(void);

#endif
