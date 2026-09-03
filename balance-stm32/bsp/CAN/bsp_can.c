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

#include "bsp_can.h"

moto_info_t motor_info[MOTOR_MAX_NUM];
uint16_t can_cnt;
volatile uint32_t can_rx_total;
volatile uint32_t can_filter_result;
volatile uint32_t can_start_result;
volatile uint32_t can_notify_result;
volatile CAN_RxHeaderTypeDef can_last_rx_header;
volatile uint8_t can_last_rx_data[8];

void can_user_init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;

  can_filter_result = HAL_CAN_ConfigFilter(hcan, &can_filter);
  can_start_result = HAL_CAN_Start(hcan);
  can_notify_result = HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  static uint8_t      angle_init[MOTOR_MAX_NUM] = {0};

  if (hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
      return;

    can_last_rx_header = rx_header;
    for (uint8_t i = 0; i < 8; i++)
      can_last_rx_data[i] = rx_data[i];
    can_rx_total++;
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;
    motor_info[index].can_id         = rx_header.StdId;
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];

    int16_t raw = (int16_t)motor_info[index].rotor_angle;
    if (angle_init[index]) {
      int16_t diff = raw - motor_info[index].last_raw_angle;
      if (diff > 4096)      diff -= 8192;
      else if (diff < -4096) diff += 8192;
      motor_info[index].total_angle += diff;
    } else {
      motor_info[index].total_angle = raw;
      angle_init[index] = 1;
    }
    motor_info[index].last_raw_angle = raw;
    can_cnt++;
  }
}

/* ---- send current (M2006 + C610) ----------------------------------------- */
void set_motor_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
  uint32_t            tx_mail;

  tx_header.StdId = CAN_CONTROL_ID_BASE;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (iq1 >> 8) & 0xFF;
  tx_data[1] =  iq1       & 0xFF;
  tx_data[2] = (iq2 >> 8) & 0xFF;
  tx_data[3] =  iq2       & 0xFF;
  tx_data[4] = (iq3 >> 8) & 0xFF;
  tx_data[5] =  iq3       & 0xFF;
  tx_data[6] = (iq4 >> 8) & 0xFF;
  tx_data[7] =  iq4       & 0xFF;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mail);
}
