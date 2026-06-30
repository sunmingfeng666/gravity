#ifndef _USART_TASK_H_
#define _USART_TASK_H_

#include "main.h"
#include "Referee.h"

//相关变量的结构体
typedef struct __packed
{
		uint8_t Switch;
		float	ARM_Position_Feedback_J1;
		float	ARM_Position_Feedback_J2;
		float	ARM_Position_Feedback_J3;
		float	ARM_Position_Feedback_J4;
		float	ARM_Position_Feedback_J5;
		float	ARM_Position_Feedback_J6;
		int8_t a[5];
} UserCtrl_t;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef FrameHeader;
        uint16_t CmdId;//命令码
        UserCtrl_t UserCtrl;//核心数据
        uint16_t CRC16;
    }channal;
    uint8_t Data[39];
}Data_Transmit_t;


//主控通讯
union COMMUNICATION_UNION_Typdef // 使用共用体整合数据
{
    struct __packed
    {
			//包头
			uint8_t Head_Pack;
			
			//数据总长度
			uint8_t Data_Length;
			
			
			float VX;//
      float VY;//
      float VW;//
			
			int32_t Picture_Lift;
			int32_t Picture_Transverse;
			
			int32_t Rock_Arm;//摇臂位置//
			float Track;//履带，控制位置
			int32_t Lead_Screw;//丝杠，控制位置
			
			uint8_t One_Click_Step;
			uint8_t One_Click_Reset;
			uint8_t Store_Sign;
			uint8_t UI_Updata;
			uint8_t Clamp;
			
			//校验和
			uint8_t Check_Sum;
			
    } DataNeaten;
		
    // 接收到的数组
    uint8_t  Data[40];
};

void Send_To_Custom();
void Send_To_Slave_Machine();
#endif
