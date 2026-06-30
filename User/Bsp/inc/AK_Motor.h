#ifndef _AK_MOTOR_H_
#define _AK_MOTOR_H_

#include "main.h"
#include "can_bsp.h"
#include "RUI_MATH.h"
#include "string.h"
#include "controller.h"


//enum __AK_ERR{

//	flaut_blank = 0,
//	flaut_overtemperature,
//	flaut_overcurrent,
//	flaut_overvoltage,
//	flaut_undervoltage,
//	flaut_encode,
//	flaut_overtemp_mos,
//	flaut_locked_rotor
//}AK_ERROR;

typedef struct __AK_DATA{

	int8_t  ONLINE_JUDGE_TIME;
	uint8_t ID;
  float Angle_last; // 上一个角度值
  float Angle_now;  // 现在的角度值
  float Speed_last; // 上一个速度值
  float Speed_now;  // 现在的速度值
	float Torque;
  int8_t  temperature;
  int32_t Angle_Infinite;//处理后连续的编码器值  
	uint8_t err_code;
	
	float error;
  float Des_Aim;
  float Des_Aim_last;
  float dt;
	
}AK_MOTOR_DATA_Typedef;

typedef struct
{
  uint8_t PID_INIT;
	AK_MOTOR_DATA_Typedef DATA;  //电机的基础数据，例如转速啥的
  Feedforward_t PID_F;   //前馈控制用于补偿系统动态特性，提高控制精度。
  PID_t PID_P;
  PID_t PID_S;
	
}AK_MOTOR_Typedef;

void AK80_8_init(FDCAN_HandleTypeDef *hcan,uint8_t ID);
void AK80_8_control(FDCAN_HandleTypeDef *hcan,AK_MOTOR_Typedef* motor,float p_des, float v_des, float kp, float kd, float t_ff);
void AK80_8_CAN_RX(AK_MOTOR_Typedef* motor,uint8_t * msg);
#endif
