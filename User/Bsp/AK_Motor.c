#include "AK_Motor.h"



/**
 * @brief  AK80_8的控制函数
 * @param  hcan     哪一路can
 * @param  ID       控制报文的ID
 * @param  p_des    目标位置
 * @param  v_des    目标速度
 * @param  t_ff     目标力矩
 * @return     
 */
void AK80_8_control(FDCAN_HandleTypeDef *hcan,AK_MOTOR_Typedef* motor,float p_des, float v_des, float kp, float kd, float t_ff) {
	
	    uint8_t tx_message[8];
	
			/// limit data to be within bounds ///
			float P_MIN =-12.5f;
			float P_MAX =12.5f;
			float V_MIN =-30.0f;
			float V_MAX =30.0f;
			float T_MIN =-18.0f;
			float T_MAX =18.0f;
			float KP_MIN =0;
			float KP_MAX =500.0f;
			float KD_MIN =0;               
			float KD_MAX =5.0f;
			float p = p_des*180.0f/PI;
			p = fminf(fmaxf(P_MIN, p), P_MAX);
			v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
			kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
			kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
			t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
	
	
			/// convert floats to unsigned ints ///
			int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
			int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
			int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
			int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
			int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
			
			
			/// pack ints into the can buffer ///
			tx_message[0] = p_int>>8; //位置高 8
			tx_message[1] = p_int&0xFF; //位置低 8
			tx_message[2] = v_int>>4; //速度高 8 位
			tx_message[3] = ((v_int&0xF)<<4)|(kp_int>>8); //速度低 4 位 KP 高 4 位
			tx_message[4] = kp_int&0xFF; //KP 低 8 位
			tx_message[5] = kd_int>>4; //Kd 高 8 位
			tx_message[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP 低 4 位扭矩高 4 位
			tx_message[7] = t_int&0xff; //扭矩低 8 位

		
	  //发送数据帧
		canx_send_data(hcan, motor->DATA.ID, tx_message, sizeof (tx_message));
}

//AK电机初始化
void AK80_8_init(FDCAN_HandleTypeDef *hcan,uint8_t ID){
	
	uint8_t tx_message[8];

    // 填充CAN数据帧
    tx_message[0] = 0xff;
    tx_message[1] = 0xff;
    tx_message[2] = 0xff;
    tx_message[3] = 0xff;
    tx_message[4] = 0xff;
    tx_message[5] = 0xff;
    tx_message[6] = 0xff;
    tx_message[7] = 0xfc;
		
		
	  //发送数据帧
		canx_send_data(hcan,ID, tx_message, sizeof (tx_message));
}

//设置零点
void AK80_set_origin(FDCAN_HandleTypeDef *hcan,AK_MOTOR_Typedef* motor){

	uint8_t tx_message[8];

    // 填充CAN数据帧
    tx_message[0] = 0xff;
    tx_message[1] = 0xff;
    tx_message[2] = 0xff;
    tx_message[3] = 0xff;
    tx_message[4] = 0xff;
    tx_message[5] = 0xff;
    tx_message[6] = 0xff;
    tx_message[7] = 0xfe;
		
		
	  //发送数据帧
		canx_send_data(hcan, motor->DATA.ID, tx_message, sizeof (tx_message));
}

//AK电机接收
void AK80_8_CAN_RX(AK_MOTOR_Typedef* motor,uint8_t * msg) {
	
    motor->DATA.ID = msg[0];  // 驱动器ID
    int p_int = (msg[1] << 8) | msg[2];
    int v_int = (msg[3] << 4) | (msg[4] >> 4);
    int i_int = ((msg[4] & 0xF) << 8) | msg[5];
    motor->DATA.temperature = msg[6] - 40;
    motor->DATA.err_code = msg[7];
	
		motor->DATA.Angle_now = (uint_to_float(p_int, -12.5f, 12.5f, 16)*180.f/PI);
		motor->DATA.Speed_now = uint_to_float(v_int, -37.5f, 37.5f, 12);
		motor->DATA.Torque = uint_to_float(i_int, -32.0f, -32.0f, 12);
}
