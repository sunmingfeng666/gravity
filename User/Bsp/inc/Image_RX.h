#ifndef __IMAGE_RX_H
#define __IMAGE_RX_H

#include "main.h"
#include "Referee.h"
#include "MY_Define.h"
#include "Motors.h"

/*0x0302*/
typedef struct __packed//30个字节
{
		int16_t Mechanical_Arm_J1;
		int16_t Mechanical_Arm_J2;
		int16_t Mechanical_Arm_J3;
		int16_t Mechanical_Arm_J4;
		int16_t Mechanical_Arm_J5;
		int16_t Mechanical_Arm_J6;
		uint8_t Clamp;//夹爪
		uint8_t Main_Switch;//总开关
		
		int8_t a[16];
} Aix6_t;


typedef union//自定义控制器
{
    struct __packed
    {
        frame_header_R_Typdef FrameHeader;
        uint16_t CmdId;//命令码
        Aix6_t Aix6;
        uint16_t CRC16;
        uint16_t ONLINE_JUDGE_TIME_0x302;
    }channal;
    uint8_t Data[35];
}Data_RX_0x302_t;



typedef union
{
	frame_header_R_Typdef FrameHeader;
	uint8_t Data[255];
}select_t;

void Read_Data(select_t *sel, Data_RX_0x302_t *RX_0x302);

#endif
