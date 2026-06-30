#include "Image_RX.h"
#include "Motors.h"
#include "All_Init.h"


//自定义控制器更新（图传链路）
void F_RX302_CONTROL(Data_RX_0x302_t *RX_0x302, CONTAL_Typedef *CONTROL)
{
	CONTROL->control[1].MECHANICAL_ARM.J1_8009p = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J1)/1000.0f;
	CONTROL->control[1].MECHANICAL_ARM.J2_8009P = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J2)/1000.0f;
	CONTROL->control[1].MECHANICAL_ARM.J3_4340 = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J3)/1000.0f;
	CONTROL->control[1].MECHANICAL_ARM.J4_4340 = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J4)/1000.0f;
	CONTROL->control[1].MECHANICAL_ARM.J5_4310 = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J5)/1000.0f;
	CONTROL->control[1].MECHANICAL_ARM.J6_4310 = ((float)RX_0x302->channal.Aix6.Mechanical_Arm_J6)/1000.0f;
	CONTROL->Terminal = RX_0x302->channal.Aix6.Clamp;//夹爪
	CONTROL->control[1].MECHANICAL_ARM.sw = RX_0x302->channal.Aix6.Main_Switch;//决定自定义控制器是否生效
}
