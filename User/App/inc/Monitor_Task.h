/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2023-12-31 13:49:55
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-10-23 09:54:59
 */
/*
____/\\\\\\\\\_____        __/\\\________/\\\_        __/\\\\\\\\\\\_
 __/\\\///////\\\___        _\/\\\_______\/\\\_        _\/////\\\///__
  _\/\\\_____\/\\\___        _\/\\\_______\/\\\_        _____\/\\\_____
   _\/\\\\\\\\\\\/____        _\/\\\_______\/\\\_        _____\/\\\_____
    _\/\\\//////\\\____        _\/\\\_______\/\\\_        _____\/\\\_____
     _\/\\\____\//\\\___        _\/\\\_______\/\\\_        _____\/\\\_____
      _\/\\\_____\//\\\__        _\//\\\______/\\\__        _____\/\\\_____
       _\/\\\______\//\\\_        __\///\\\\\\\\\/___        __/\\\\\\\\\\\_
        _\///________\///__        ____\/////////_____        _\///////////__
*/
#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H
#include "main.h"
#include "stdint.h"
#include "RUI_MATH.h"
#include "MY_define.h"
#include "RUI_DBUS.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "ws2812.h"
#include "Image_RX.h"
#include "Motors.h"
#include "stdbool.h"

//在定时器调用这一个函数即可
void RUI_F_ROOT(STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typdef *MOTOR);
//总的状态监测
void RUI_F_ROOT_ALL_STATUS(STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typdef *MOTOR);
//主控位置判断
uint8_t RUI_F_MASTER_LOCATION_JUDGE(STATUS_Typedef *Root);
//主控通信
//uint8_t RUI_F_MASTER_COMMUNICATION(void);
//LED
void RUI_F_LED_SHOW_STATUS(STATUS_Typedef *Root);
uint8_t Monitor_FDCAN(FDCAN_HandleTypeDef *hcan);
#endif

