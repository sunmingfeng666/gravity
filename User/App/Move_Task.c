#include "Move_Task.h" 
#include "One_Click_Task.h"
#include "tim.h"


#define Mechanical_arm_Drag 0  //用手可拖动

#define Boundary_Up_J1 			0.0
#define Boundary_Down_J1	  0.0
#define Boundary_Up_J2 			3.67875862
#define Boundary_Down_J2 		-0.532349
#define Boundary_Up_J4 		 	1.848
#define Boundary_Down_J4 		-1.8297427
#define Boundary_Up_J5 			1.67410f
#define Boundary_Down_J5 		-1.76146317f
#define Boundary_Up_Transverse 				0
#define Boundary_Down_Transverse 		  -624358
#define Boundary_Up_Lift 				1025000
#define Boundary_Down_Lift		  0

uint8_t PlanTrajectory = 0;
uint8_t power_on_arm = 0;
float all_control_1;
float all_control_2;
float all_control_3;
float all_control_4;
float all_control_5;
float all_control_6;

uint32_t Set_Servo_Angle(float angle, float max_angle) {
    // 角度安全限制
    if (angle < 0)      angle = 0;
    if (angle > max_angle) angle = max_angle;
    
    // 计算角度对应的比例 (0.0 ~ 1.0)
    float ratio = angle / max_angle;
    
    /* 
     * 修正后的计算：1 Tick = 1 μs
     * 0°  -> 0.5ms = 500 μs = 500 计数值
     * max_angle -> 2.5ms = 2500 μs = 2500 计数值
     * 脉冲宽度范围 = 500 ~ 2500 (共2000计数)
     */
    uint32_t pulse_count = 500 + (uint32_t)(ratio * 2000.0f);
    
		return pulse_count;		
}
//--------
void Motor_Soft_Start()
{
	ALL_CONTROL.Slave.VX = ALL_CONTROL.control[0].BOTTOM.VX + ALL_CONTROL.control[1].BOTTOM.VX + ALL_CONTROL.control[2].BOTTOM.VX + ALL_CONTROL.control[3].BOTTOM.VX;
	ALL_CONTROL.Slave.VY = ALL_CONTROL.control[0].BOTTOM.VY + ALL_CONTROL.control[1].BOTTOM.VY + ALL_CONTROL.control[2].BOTTOM.VY + ALL_CONTROL.control[3].BOTTOM.VY;
	ALL_CONTROL.Slave.VW = ALL_CONTROL.control[0].BOTTOM.VW + ALL_CONTROL.control[1].BOTTOM.VW + ALL_CONTROL.control[2].BOTTOM.VW + ALL_CONTROL.control[3].BOTTOM.VW;
	ALL_CONTROL.Slave.Picture_Lift = ALL_CONTROL.control[0].PICTURE.Picture_Lift + ALL_CONTROL.control[1].PICTURE.Picture_Lift + ALL_CONTROL.control[2].PICTURE.Picture_Lift +ALL_CONTROL.control[3].PICTURE.Picture_Lift;
	ALL_CONTROL.Slave.Picture_Transverse = ALL_CONTROL.control[0].PICTURE.Picture_Transverse + ALL_CONTROL.control[1].PICTURE.Picture_Transverse + ALL_CONTROL.control[2].PICTURE.Picture_Transverse +ALL_CONTROL.control[3].PICTURE.Picture_Transverse;
	ALL_CONTROL.Slave.Track = ALL_CONTROL.control[0].STEPS.Track + ALL_CONTROL.control[1].STEPS.Track + ALL_CONTROL.control[2].STEPS.Track +ALL_CONTROL.control[3].STEPS.Track;
	ALL_CONTROL.Slave.Rock_Arm = ALL_CONTROL.control[0].STEPS.Rock_Arm + ALL_CONTROL.control[1].STEPS.Rock_Arm + ALL_CONTROL.control[2].STEPS.Rock_Arm +ALL_CONTROL.control[3].STEPS.Rock_Arm;
	ALL_CONTROL.Slave.Lead_Screw = ALL_CONTROL.control[0].STEPS.Lead_Screw + ALL_CONTROL.control[1].STEPS.Lead_Screw + ALL_CONTROL.control[2].STEPS.Lead_Screw +ALL_CONTROL.control[3].STEPS.Lead_Screw;
	
	
	
	/*	防止运动开始与结尾的速度突变，保证操作手手感
	* 	|y  ______
	* 	|  /      \
	* 	| /        \
	* 	+------------x
	*/
	static float SLOW_START_CHASSIS = 0.0f;
	static float SLOW_START_PICTURE_Transverse = 0.0f;
	
		
	//以速度是否等于0为界
	if( (ALL_CONTROL.Slave.VX + ALL_CONTROL.Slave.VY+ ALL_CONTROL.Slave.VW) != 0)
	{
		SLOW_START_CHASSIS += ALL_CONTROL.Slave.Soft_Start_Chassis;//增长速度
		
		if (SLOW_START_CHASSIS > 1.0f){SLOW_START_CHASSIS = 1.0f ;}
  } else{
		
    SLOW_START_CHASSIS -= ALL_CONTROL.Slave.Soft_Start_Chassis * 4.0f; // 刹车稍微比缓慢起步快一点，保证手感
    if (SLOW_START_CHASSIS < 0.0f) SLOW_START_CHASSIS = 0.0f;
  }
	
	
	if(ALL_CONTROL.Mode == Pose_Take)
	{
		ALL_CONTROL.Slave.VY *= SLOW_START_CHASSIS;
	}else if(ALL_CONTROL.Mode == Normal)
	{
		
	}

	
	ALL_CONTROL.Slave.VX *= SLOW_START_CHASSIS;
  //ALL_CONTROL.Slave.VY *= SLOW_START_CHASSIS;
	//ALL_CONTROL.Slave.VW *= SLOW_START_CHASSIS;
}
//-----
void Motion_limitation()
{
				//关节2限幅
				if(all_control_2 <= Boundary_Down_J2 || all_control_2 >= Boundary_Up_J2)
				{
					if(all_control_2 <= Boundary_Down_J2)
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P = Boundary_Down_J2*ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P = Boundary_Down_J2*ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = Boundary_Down_J2*ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P = Boundary_Down_J2*ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P/all_control_2;
					}else
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P = Boundary_Up_J2*ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P = Boundary_Up_J2*ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = Boundary_Up_J2*ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P/all_control_2;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P = Boundary_Up_J2*ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P/all_control_2;
					}
					all_control_2 = ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P + ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P + ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P;
				}
				//关节4限幅
				if(all_control_4 <= Boundary_Down_J4 || all_control_4 >= Boundary_Up_J4)
				{
					if(all_control_4 <= Boundary_Down_J4)
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 = Boundary_Down_J4 * ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340 = Boundary_Down_J4 * ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = Boundary_Down_J4 * ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340 = Boundary_Down_J4 * ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340 / all_control_4;
					}else
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 = Boundary_Up_J4 * ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340 = Boundary_Up_J4 * ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = Boundary_Up_J4 * ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 / all_control_4;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340 = Boundary_Up_J4 * ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340 / all_control_4;
					}
					all_control_4 = ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 + ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 + ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340;
				}
				//关节5限幅
				if(all_control_5 <= Boundary_Down_J5 || all_control_5 >= Boundary_Up_J5)
				{
					if(all_control_5 <= Boundary_Down_J5)
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 = Boundary_Down_J5 * ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310 = Boundary_Down_J5 * ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = Boundary_Down_J5 * ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310 = Boundary_Down_J5 * ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310 / all_control_5;
					}else
					{
						ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 = Boundary_Up_J5 * ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310 = Boundary_Up_J5 * ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = Boundary_Up_J5 * ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 / all_control_5;
						ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310 = Boundary_Up_J5 * ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310 / all_control_5;
					}
					all_control_5 = ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 + ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 + ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310;
				}
				
//				//图传横移
//				if(ALL_CONTROL.Slave.Picture_Transverse <= Boundary_Down_Transverse || ALL_CONTROL.Slave.Picture_Transverse >= Boundary_Up_Transverse)
//				{
//					if(ALL_CONTROL.Slave.Picture_Transverse <= Boundary_Down_Transverse)
//					{
//						ALL_CONTROL.control[3].PICTURE.Picture_Transverse = Boundary_Down_Transverse * ALL_CONTROL.control[3].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[2].PICTURE.Picture_Transverse = Boundary_Down_Transverse * ALL_CONTROL.control[2].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[1].PICTURE.Picture_Transverse = Boundary_Down_Transverse * ALL_CONTROL.control[1].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[0].PICTURE.Picture_Transverse = Boundary_Down_Transverse * ALL_CONTROL.control[0].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//					}else
//					{
//						ALL_CONTROL.control[3].PICTURE.Picture_Transverse = Boundary_Up_Transverse * ALL_CONTROL.control[3].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[2].PICTURE.Picture_Transverse = Boundary_Up_Transverse * ALL_CONTROL.control[2].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[1].PICTURE.Picture_Transverse = Boundary_Up_Transverse * ALL_CONTROL.control[1].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//						ALL_CONTROL.control[0].PICTURE.Picture_Transverse = Boundary_Up_Transverse * ALL_CONTROL.control[0].PICTURE.Picture_Transverse / ALL_CONTROL.Slave.Picture_Transverse;
//					}
//					ALL_CONTROL.Slave.Picture_Transverse = ALL_CONTROL.control[3].PICTURE.Picture_Transverse + ALL_CONTROL.control[2].PICTURE.Picture_Transverse
//														 +ALL_CONTROL.control[1].PICTURE.Picture_Transverse + ALL_CONTROL.control[0].PICTURE.Picture_Transverse;
//				}
//				
//				//图传抬升
//				if(ALL_CONTROL.Slave.Picture_Lift <= Boundary_Down_Lift || ALL_CONTROL.Slave.Picture_Lift >= Boundary_Up_Lift)
//				{
//					if(ALL_CONTROL.Slave.Picture_Lift <= Boundary_Down_Lift)
//					{
//						ALL_CONTROL.control[3].PICTURE.Picture_Lift = Boundary_Down_Lift * ALL_CONTROL.control[3].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[2].PICTURE.Picture_Lift = Boundary_Down_Lift * ALL_CONTROL.control[2].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[1].PICTURE.Picture_Lift = Boundary_Down_Lift * ALL_CONTROL.control[1].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[0].PICTURE.Picture_Lift = Boundary_Down_Lift * ALL_CONTROL.control[0].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//					}else
//					{
//						ALL_CONTROL.control[3].PICTURE.Picture_Lift = Boundary_Up_Lift * ALL_CONTROL.control[3].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[2].PICTURE.Picture_Lift = Boundary_Up_Lift * ALL_CONTROL.control[2].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[1].PICTURE.Picture_Lift = Boundary_Up_Lift * ALL_CONTROL.control[1].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//						ALL_CONTROL.control[0].PICTURE.Picture_Lift = Boundary_Up_Lift * ALL_CONTROL.control[0].PICTURE.Picture_Lift / ALL_CONTROL.Slave.Picture_Lift;
//					}
//					ALL_CONTROL.Slave.Picture_Lift = ALL_CONTROL.control[3].PICTURE.Picture_Lift + ALL_CONTROL.control[2].PICTURE.Picture_Lift
//														 +ALL_CONTROL.control[1].PICTURE.Picture_Lift + ALL_CONTROL.control[0].PICTURE.Picture_Lift;
//				}
}

//------
uint8_t Motor_MoveControl()
{
	
	if(ALL_CONTROL.Terminal ==Clamp_Close) //闭合
	{
		Motors.DM_3507_Terminal.MIT.torque_des = 1.0f;
	}else{
				
		Motors.DM_3507_Terminal.MIT.torque_des = -1.2f;
	}

	
	
	#if Mechanical_arm_Drag
			
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x01,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);
			
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x02,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);

					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x03,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);

					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x04,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);

					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x05,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);

					//ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x06,POS_MODE, DM_CMD_MOTOR_MODE);
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x06,POS_MODE, DM_CMD_RESET_MODE);
					osDelay(2);
	        
					if(VT13.Remote.pause == 1 )
					{ 
						ALL_CONTROL.Terminal =Clamp_Open;
					}else{ALL_CONTROL.Terminal =Clamp_Close;} 
	
	
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x07,MIT_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
									
					ALL_STATUS.STATUS_FDCAN_3 = mit_ctrl(&hfdcan3, 0x07, &Motors.DM_3507_Terminal.MIT);
					osDelay(2);
					
					if(VT13.Remote.mode_sw == 2) 
					{ 
						PlanTrajectory = 1;
					}else{
					
						PlanTrajectory = 0;
					}
					One_Click_PlanTrajectory(PlanTrajectory);
//			
		#else
			
			if( power_on_arm == 1 )
			{
				all_control_1 = ALL_CONTROL.control[3].MECHANICAL_ARM.J1_8009p + ALL_CONTROL.control[2].MECHANICAL_ARM.J1_8009p
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J1_8009p + ALL_CONTROL.control[0].MECHANICAL_ARM.J1_8009p;
				all_control_2 = ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P + ALL_CONTROL.control[2].MECHANICAL_ARM.J2_8009P
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P + ALL_CONTROL.control[0].MECHANICAL_ARM.J2_8009P;
				all_control_3 = ALL_CONTROL.control[3].MECHANICAL_ARM.J3_4340 + ALL_CONTROL.control[2].MECHANICAL_ARM.J3_4340
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J3_4340 + ALL_CONTROL.control[0].MECHANICAL_ARM.J3_4340;
				all_control_4 = ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 + ALL_CONTROL.control[2].MECHANICAL_ARM.J4_4340
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 + ALL_CONTROL.control[0].MECHANICAL_ARM.J4_4340;
				all_control_5 = ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 + ALL_CONTROL.control[2].MECHANICAL_ARM.J5_4310
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 + ALL_CONTROL.control[0].MECHANICAL_ARM.J5_4310;
				all_control_6 = ALL_CONTROL.control[3].MECHANICAL_ARM.J6_4310 + ALL_CONTROL.control[2].MECHANICAL_ARM.J6_4310
														 +ALL_CONTROL.control[1].MECHANICAL_ARM.J6_4310 + ALL_CONTROL.control[0].MECHANICAL_ARM.J6_4310;
				
				Motion_limitation();
				
				//将控制指令（一键+图传链路+大白）的数据附注到电机目标值中
				Motors.DM_8009_J1.PV.POS = all_control_1;
				Motors.DM_8009_J2.PV.POS = all_control_2;
				Motors.DM_4340_J3.PV.POS = all_control_3;
				Motors.DM_4340_J4.PV.POS = all_control_4;
				Motors.DM_4310_J5.PV.POS = all_control_5;
				Motors.DM_4310_J6.PV.POS = all_control_6;
					
				

				if(Motors.DM_8009_J1.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x01,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x01,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				if(Motors.DM_8009_J2.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x02,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x02,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				if(Motors.DM_4340_J3.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x03,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_2 = motor_mode(&hfdcan2, 0x03,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				if(Motors.DM_4340_J4.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x04,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x04,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				if(Motors.DM_4310_J5.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x05,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x05,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				if(Motors.DM_4310_J6.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x06,POS_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x06,POS_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
				  if(Motors.DM_3507_Terminal.DATA.ONLINE_JUDGE_TIME ==0) {
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x07,MIT_MODE, DM_CMD_CLEAR_ERROR);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_3 = motor_mode(&hfdcan3, 0x07,MIT_MODE, DM_CMD_MOTOR_MODE);
					osDelay(2);
				}
		
				Whether_Perform_Oneclick();
			
			
				//只有所以电机都在线时才发控制帧
				if(ALL_STATUS.STATUS_ARM_8009_1 			 	 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_8009_2 				 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_4340_ROLL 			 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_4340_PITCH_1 	 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_4310_ROLL 	  	 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_4310_PITCH_2 	 == RUI_DF_ONLINE &&
					 ALL_STATUS.STATUS_ARM_3507_TERMINAL	 == RUI_DF_ONLINE  )
				{
					ALL_STATUS.STATUS_FDCAN_2 = pos_speed_ctrl(&hfdcan2, 0x01, &Motors.DM_8009_J1.PV);	
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_2 = pos_speed_ctrl(&hfdcan2, 0x02, &Motors.DM_8009_J2.PV);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_2 = pos_speed_ctrl(&hfdcan2, 0x03, &Motors.DM_4340_J3.PV);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_1 = pos_speed_ctrl(&hfdcan3, 0x04, &Motors.DM_4340_J4.PV);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_1 = pos_speed_ctrl(&hfdcan3, 0x05, &Motors.DM_4310_J5.PV);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_1 = pos_speed_ctrl(&hfdcan3, 0x06, &Motors.DM_4310_J6.PV);
					osDelay(2);
					ALL_STATUS.STATUS_FDCAN_1 = mit_ctrl(&hfdcan3, 0x07, &Motors.DM_3507_Terminal.MIT);
					osDelay(2);
				}
				
			
			}else//保证上电缓慢，防止冲击过大
			{
				if(fabs(Motors.DM_8009_J2.DATA.pos) < 0.1) 
				{
					power_on_arm = 1;
					
					Motors.DM_8009_J1.PV.SPE = 2.0;
					Motors.DM_8009_J2.PV.SPE = 2.0;
					Motors.DM_4340_J3.PV.SPE = 10.0;
					Motors.DM_4340_J4.PV.SPE = 10.0;
					Motors.DM_4310_J5.PV.SPE = 10.0;
					Motors.DM_4310_J6.PV.SPE = 10.0;
				}
			}
			#endif
			
			
		return RUI_DF_READY;
}

uint8_t Motor_MoveProtect( MOTOR_Typdef *MOTOR , STATUS_Typedef *Root)
{

//	/*遥控离线保护*/
//  if( Root->RM_DBUS == RUI_DF_OFFLINE )
//  {
//  }
	
	return RUI_DF_READY;
}
