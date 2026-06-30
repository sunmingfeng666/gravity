#ifndef __ALL_INIT_H
#define __ALL_INIT_H

#include "can_bsp.h"
#include "bsp_dwt.h"
#include "tim.h"
#include "usart.h"
#include "RUI_DBUS.h"
#include "Referee.h"
#include "Image_RX.h"
#include "cmsis_os.h"
#include "Monitor_Task.h"
#include "MY_Define.h"
#include "fdcan.h"
#include "DJI_Motor.h"
#include "IMU_Task.h"
#include "ws2812.h"
#include "string.h"
#include "RUI_MATH.h"
#include "adc.h"
#include "bsp_buzzer.h"
#include "DM_Motor.h"
#include "Gimbal_Task.h"
#include "Motors.h"
#include "usbd_cdc_if.h"
#include "NIMING.h"
#include "K3debug_Task.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "VOFA.h"
#include "DM_test.h"
#include "Move_Task.h" 
#include "bottom.h"
#include "AK_Motor.h"
#include "Usart_Task.h"
#include "VT13.h"
#include "Circular_Buffer.h"

void Everying_Init(void);

//extern uint8_t DBUS_RX_DATA[18] __attribute__((section(".ARM.__at_0x24000000")));
extern __attribute__((section (".AXI_SRAM"))) uint8_t DBUS_RX_DATA[18] ;
extern DBUS_Typedef WHW_V_DBUS;
extern VT13_Typedef VT13;
extern CONTAL_Typedef RUI_V_CONTAL;

extern ALL_RX_Data_T ALL_RX;
extern User_Data_T User_data;

extern uint8_t RX[20];

extern MOTOR_Typdef Motors;

extern struct mecanumInit_typdef mecanumInit_t;

extern FuzzyRule_t fuzzyRules;

extern PID_t temppid;

extern IMU_Data_t IMU_Data;

extern V_Input_t V_Input;

extern STATUS_Typedef ALL_STATUS;

extern __attribute__((section (".AXI_SRAM"))) select_t selects;
extern Data_RX_0x302_t Data_RX_0x302;

extern CONTAL_Typedef ALL_CONTROL;

extern union COMMUNICATION_UNION_Typdef Communication;

extern uint8_t PID_INIT ;
extern uint8_t AIM_INIT ;

extern float PID_F_Chassis[3];
extern float PID_S_Chassis[3] ;

extern float RX_temporary [3];
extern uint8_t state_master;
extern select_t wuxianchuankou;
extern uint8_t ReadBuffer_Picture[156];//图传第一次缓冲
extern Circular_TypeDef Circular_Buffer_Picture;
#endif
