#include "HID_Control.h"

uint16_t V_Chassis_VX = 650;
uint16_t V_Chassis_VY = 930;
uint16_t V_Chassis_VW1 = 80;
uint16_t V_Chassis_VW2 = 5;
uint16_t V_Picture_Transverse = 3600;
float KV = 1.5;

void HID_Control(VT13_Typedef  HID_Data, CONTAL_Typedef *control)
{
	
	//static uint8_t shift_v_flag = 0;
	
	//方向
	int8_t Direction_VX_N = HID_Data.KeyBoard.D - HID_Data.KeyBoard.A;
	int8_t Direction_VY_N = HID_Data.KeyBoard.W - HID_Data.KeyBoard.S;
	int8_t Direction_VY_Arm = HID_Data.KeyBoard.D - HID_Data.KeyBoard.A;
	int8_t Direction_VX_Arm = -HID_Data.KeyBoard.W + HID_Data.KeyBoard.S;
	
	int8_t Direction_VW1 = HID_Data.KeyBoard.E - HID_Data.KeyBoard.Q;
	int8_t Direction_Transverse = HID_Data.KeyBoard.R - HID_Data.KeyBoard.F;
	
	//shift加速,Ctrl减速
	if(HID_Data.KeyBoard.Shift == 1)
	{
		V_Chassis_VY = 1200;
	}else if(HID_Data.KeyBoard.Ctrl == 1){
	
		V_Chassis_VY = 600;
	}else 
	{
		V_Chassis_VY = 930;
	}
	
	int8_t a = HID_Data.KeyBoard.Z + 2*HID_Data.KeyBoard.X + 4*HID_Data.KeyBoard.C + 8*HID_Data.KeyBoard.V;
	
	
	if(HID_Data.KeyBoard.Ctrl == 0)
	{
		if(HID_Data.KeyBoard.Shift == 1)//Shift +字母：一键
		{
			switch(a)
			{
				case 1://Z
					if(control->Mode == Pose_Take || control->Mode == Custom_Control) {control->One_Click_Store = 1;}
					break;
				case 2://X
					if(control->Mode == Pose_Take || control->Mode == Custom_Control) {control->One_Click_take = 1;}
					break;
				case 4://C
					if(control->Mode == Normal) {control->One_Click_Step = 1;}
					break;
				case 8://v
//					if(shift_v_flag == 0) // 如果之前没按过(刚按下)
//					{
//						control->One_Click_Self_Prosurant ++; // 自增
//						if(control->One_Click_Self_Prosurant > 2) 
//						{
//							control->One_Click_Self_Prosurant = 0; // 超过2归零
//						}
//						shift_v_flag = 1; // 标记为已触发，不松手就不再进这个if
//					}
					break;
			}
		}
	}else{
		
		//保证一键的运行
		control->One_Click_Store = 0;
		control->One_Click_take = 0;
		control->One_Click_Step = 0;
		
		
		//Ctrl + 字母：模式切换
		switch(a)
		{
			case 1://z
				control->Mode = Custom_Control;
				break;
			case 2://x
				control->Mode = Pose_Take;//取姿态
				break;
			case 4://c
				control->Mode = Normal;//正常开车姿态
				break;
			case 8://v
				control->Mode = Selt_Prosurant;//自保模式
				break;
		}	
			
	}
	
//	// 如果不满足 (Ctrl没按 且 Shift按下 且 只有V按下) 的条件，说明手松开了，清空标志位
//	if (!(HID_Data.KeyBoard.Ctrl == 0 && HID_Data.KeyBoard.Shift == 1 && a == 8))
//	{
//		shift_v_flag = 0; 
//	}
//	
//	
	//根据模式改变底盘朝向
	if(control->Mode == Pose_Take || control->Mode == Custom_Control)
	{
		control->control[2].BOTTOM.VY = Direction_VY_Arm * 350 * KV;
		control->control[2].BOTTOM.VX = Direction_VX_Arm * 350 * KV;
	}else if(control->Mode == Normal || control->Mode == Selt_Prosurant){
	
		control->control[2].BOTTOM.VY = V_Chassis_VY * Direction_VY_N * KV;
		control->control[2].BOTTOM.VX = Direction_VX_N * V_Chassis_VX * KV;
	}
	
	control->control[2].BOTTOM.VW = V_Chassis_VW1 * Direction_VW1 + HID_Data.Mouse.X_Flt*V_Chassis_VW2;
	control->control[2].PICTURE.Picture_Transverse += Direction_Transverse * V_Picture_Transverse;
	
		
}