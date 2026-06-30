#ifndef __MOVE_TASK_H
#define __MOVE_TASK_H

#include "RUI_DBUS.h" 
#include "main.h" 
#include "Motors.h" 
#include "DJI_Motor.h" 
#include "All_Init.h" 
#include "Bottom.h"


// 麦克纳姆轮初始化结构体
struct mecanumInit_typdef
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float wheeltrack;         /* 轮距（mm）*/
    float wheelbase;          /* 轴距（mm）*/
    float rotate_x_offset;    /* 相对于底盘中心的x轴旋转偏移量(mm) */
    float rotate_y_offset;    /* 相对于底盘中心的y轴旋转偏移量(mm) */
    float deceleration_ratio; /* 电机减速比 */
    int max_vx_speed;         /* 底盘的x轴的最大速度(mm/s) */
    int max_vy_speed;         /* 底盘的y轴的最大速度(mm/s) */
    int max_vw_speed;         /* 底盘的自转的最大速度(degree/s) */
    int max_wheel_ramp;       /* 3508最大转速 */
                              // 每一个轮子的旋转比率//与旋转中心点相关
    float raid_fr;            // 右前
    float raid_fl;            // 左前
    float raid_bl;            // 左后
    float raid_br;            // 右后
    float wheel_rpm_ratio;    // 用来将速度转化成转每分钟
};

uint8_t Motor_MoveControl();
uint8_t Motor_MoveProtect( MOTOR_Typdef *MOTOR, STATUS_Typedef *Root);
uint32_t Set_Servo_Angle(float angle, float max_angle) ;
void Motor_Soft_Start();

void F_RX302_CONTROL(Data_RX_0x302_t *RX_0x302, CONTAL_Typedef *CONTROL);
#endif