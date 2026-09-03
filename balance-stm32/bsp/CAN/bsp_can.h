/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

/* ---- M2006 + C610 CAN ID assignment ------------------------------------- */
#define FEEDBACK_ID_BASE     0x201
#define CAN_CONTROL_ID_BASE  0x200

#define MOTOR_MAX_NUM          7

/* ---- per-motor data ------------------------------------------------------ */
typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
    int32_t  total_angle;
    int16_t  last_raw_angle;
} moto_info_t;

extern moto_info_t motor_info[];
extern uint16_t    can_cnt;
extern volatile uint32_t can_rx_total;
extern volatile uint32_t can_filter_result;
extern volatile uint32_t can_start_result;
extern volatile uint32_t can_notify_result;
extern volatile CAN_RxHeaderTypeDef can_last_rx_header;
extern volatile uint8_t can_last_rx_data[8];

/* ---- API ---------------------------------------------------------------- */
void can_user_init(CAN_HandleTypeDef *hcan);
void set_motor_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
