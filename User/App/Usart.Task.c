#include "Usart_Task.h"
#include "All_Init.h"
#include "RUI_MATH.h"

//串口发送(机器人->自定义控制器)
//Data_Transmit_t Data_Transmit = {0};

//void Send_To_Custom()
//{
//		Data_Transmit.channal.FrameHeader.SOF = 0xA5;
//    Data_Transmit.channal.FrameHeader.DataLenth = 30;
//    Data_Transmit.channal.FrameHeader.Seq += 0x01;
//    Append_CRC8_Check_Sum(Data_Transmit.Data,5);

//    Data_Transmit.channal.CmdId = 0x309;

//    //具体的控制
//	
//		Data_Transmit.channal.UserCtrl.Switch = (ALL_CONTROL.One_Click_Store == 1 || ALL_CONTROL.One_Click_take == 1)?1:0;
//	
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J1 = Motors.DM_8009_J1.DATA.pos;
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J2 = Motors.DM_8009_J2.DATA.pos;
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J3 = Motors.DM_4340_J3.DATA.pos;
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J4 = Motors.DM_4340_J4.DATA.pos;
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J5 = Motors.DM_4310_J5.DATA.pos;
//		Data_Transmit.channal.UserCtrl.ARM_Position_Feedback_J6 = Motors.DM_4310_J6.DATA.pos;


//    Append_CRC16_Check_Sum(Data_Transmit.Data,39);
//	  HAL_UART_Transmit_IT(&huart1,Data_Transmit.Data,39);
//	
//}


//双板通讯（串口）
void Send_To_Slave_Machine()
{
	Communication.DataNeaten.Head_Pack = 0xA5;
	Communication.DataNeaten.Data_Length = sizeof(Communication.Data);

	Communication.DataNeaten.VX = ALL_CONTROL.Slave.VX;
	Communication.DataNeaten.VY = ALL_CONTROL.Slave.VY;
	Communication.DataNeaten.VW = ALL_CONTROL.Slave.VW;
	Communication.DataNeaten.Picture_Lift = ALL_CONTROL.Slave.Picture_Lift;
	Communication.DataNeaten.Picture_Transverse = ALL_CONTROL.Slave.Picture_Transverse;
	Communication.DataNeaten.Rock_Arm = ALL_CONTROL.Slave.Rock_Arm;
	Communication.DataNeaten.Track = ALL_CONTROL.Slave.Track;
	Communication.DataNeaten.Lead_Screw = ALL_CONTROL.Slave.Lead_Screw;
	Communication.DataNeaten.Store_Sign = ALL_CONTROL.Store_Sign;
	Communication.DataNeaten.One_Click_Step = ALL_CONTROL.One_Click_Step;
	Communication.DataNeaten.One_Click_Reset = ALL_CONTROL.One_Click_Reset;
	Communication.DataNeaten.UI_Updata = VT13.KeyBoard.C;
	Communication.DataNeaten.Clamp = ALL_CONTROL.Terminal;
	
	// 先计算校验和，再设置校验和字段
	uint8_t temp_checksum = 0;
	Communication.DataNeaten.Check_Sum = 0; // 临时设为0以计算正确的校验和

	for(int8_t i = 0; i < Communication.DataNeaten.Data_Length - 1; i++)
	{
    temp_checksum += Communication.Data[i];
	}

	// 最后设置真正的校验和
	Communication.DataNeaten.Check_Sum = temp_checksum; 
	HAL_UART_Transmit_IT(&huart10,Communication.Data,sizeof(Communication.Data));
	
	if(Communication.DataNeaten.One_Click_Step == 1) {ALL_CONTROL.One_Click_Step = 0;}
	if(Communication.DataNeaten.One_Click_Reset == 1) {ALL_CONTROL.One_Click_Reset = 0;}
}