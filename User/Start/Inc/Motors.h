#ifndef __MOTORS_H
#define __MOTORS_H

#include "DJI_Motor.h"
#include "DM_Motor.h"


#define Clamp_Open 1
#define Clamp_Close 0


typedef struct
{
	/*机械臂*/
	DM_MOTOR_Typdef    DM_8009_J1;
	DM_MOTOR_Typdef    DM_8009_J2;
	DM_MOTOR_Typdef    DM_4340_J3;
	DM_MOTOR_Typdef    DM_4340_J4;
	DM_MOTOR_Typdef    DM_4310_J5;
	DM_MOTOR_Typdef    DM_4310_J6;
	DM_MOTOR_Typdef    DM_3507_Terminal;
	
	
	/*测试*/
	DM_MOTOR_Typdef    ceshiDM_4310;
	
}MOTOR_Typdef;

//---------------------------

typedef struct
{
    /******************************主控位置********************************/
    uint8_t MASTER_LOCATION;	
	  /******************************运行模式********************************/
    uint8_t RM_MOD;
		/******************************控制器********************************/
	  uint8_t RM_DBUS;
    uint8_t RM_Image;
		/******************************FDCAN总线********************************/
    // 返回值说明：
		// 0: 发送成功 (成功放入FIFO)
		// 1: FIFO已满，发送失败 (可能是总线故障导致数据发不出去)
		// 2: 硬件添加消息失败
		// 3: 数据长度参数错误
		//
		enum __fdcan
		{
			Success = 0,
			Full_FIFO,
			Failed_Add_FIFO,
			Wrong_Length
		}STATUS_FDCAN_1,STATUS_FDCAN_2,STATUS_FDCAN_3;
  	/******************************机械臂电机********************************/
		uint8_t STATUS_ARM_8009_1;
	  uint8_t STATUS_ARM_8009_2;
		uint8_t STATUS_ARM_4340_ROLL;
		uint8_t STATUS_ARM_4340_PITCH_1;
		uint8_t STATUS_ARM_4310_PITCH_2;
		uint8_t STATUS_ARM_4310_ROLL;
		uint8_t STATUS_ARM_3507_TERMINAL;
		/******************************存储标志********************************/
		uint8_t STATUS_STORE[4];
		/******************************一键********************************/
		uint8_t STATUS_ONE_CLICK_STORE;
		uint8_t STATUS_ONE_CLICK_TAKE;
		uint8_t STATUS_ONE_CLICK_STEP;
		uint8_t STATUS_ONE_CLICK_SELF_PROSUARANT_1;
		uint8_t STATUS_ONE_CLICK_SELF_PROSUARANT_2;
		//
		uint8_t STATUS_ModeChange_Normal_To_Pose_Take;
		uint8_t STATUS_ModeChange_Pose_Take_To_Normal;

	
}STATUS_Typedef;
//-------------------------------

typedef struct
{
		struct
    {
        float VX;
        float VY;
        float VW;
    } BOTTOM;//底盘
		
		struct
    {
        float J1_8009p;
        float J2_8009P;
        float J3_4340;
        float J4_4340;
        float J5_4310;
				float J6_4310;
				uint8_t sw;
		} MECHANICAL_ARM;//机械臂
		
		struct
    {
				double Rock_Arm;
				int32_t Track;
				int32_t Lead_Screw;
    } STEPS;//登岛
		
		struct
    {
				int32_t Picture_Lift;
				int32_t Picture_Transverse;
			
    } PICTURE;//图传
		
}__CONTAL_Typedef;

//主控制结构体
typedef struct
{
	int8_t  Front_of_the_vehicle_facing; //0表示摇臂方向，1表示丝杆方向
	
	enum __Mode
	{
		Normal = 0,//也算全局变量的一种
		Pose_Take,
		Custom_Control,
		Custom_Pairing,
		Selt_Prosurant
		}Mode,LastMode;

	int8_t Terminal;//末端力矩
	uint8_t Store_Sign;//控制存矿转盘的旋转位置
	uint8_t One_Click_Store;
	uint8_t One_Click_take;
	uint8_t One_Click_Step;
	uint8_t One_Click_Reset;
	uint8_t One_Click_Self_Prosurant;
	uint8_t ModeChange_Normal_To_Pose_Take;
	uint8_t ModeChange_Pose_Take_To_Normal;
	int16_t Servo_Yaw;
	int16_t Servo_Pitch;
		
	struct
    {
			float VX;
			float VY;
			float VW;
			
			int64_t Picture_Lift;
			int64_t Picture_Transverse;
			
			int16_t Picture_Yaw;
			int16_t Picture_Pitch;
			
			float   Rock_Arm;//摇臂位置
			int64_t Track;//履带，控制位置
			int64_t Lead_Screw;//丝杠，控制位置
			
			float Soft_Start_Chassis ;
			float Soft_Start_Picture ;
			float Soft_Start_Steps ;
		} Slave;//从机
		
  __CONTAL_Typedef control[4];//大白+自定义控制器+键鼠+灰控
		
}CONTAL_Typedef;

#endif
