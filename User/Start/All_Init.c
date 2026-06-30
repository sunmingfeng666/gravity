/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-08 17:08:38
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-14 20:38:45
 */
/*
 *                   ___====-_  _-====___
 *             _--^^^#####//      \\#####^^^--_
 *          _-^##########// (    ) \\##########^-_
 *         -############//  |\^^/|  \\############-
 *       _/############//   (@::@)   \############\_
 *      /#############((     \\//     ))#############\
 *     -###############\\    (oo)    //###############-
 *    -#################\\  / VV \  //#################-
 *   -###################\\/      \//###################-
 *  _#/|##########/\######(   /\   )######/\##########|\#_
 *  |/ |#/\#/\#/\/  \#/\##\  |  |  /##/\#/  \/\#/\#/\#| \|
 *  `  |/  V  V  `   V  \#\| |  | |/#/  V   '  V  V  \|  '
 *     `   `  `      `   / | |  | | \   '      '  '   '
 *                      (  | |  | |  )
 *                     __\ | |  | | /__
 *                    (vvv(VVV)(VVV)vvv)
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                神兽保佑            永无BUG
 */


#include "All_Init.h"
//***********************************************************************************************************
__attribute__((section (".AXI_SRAM"))) uint8_t DBUS_RX_DATA[18] ;
DBUS_Typedef WHW_V_DBUS;
VT13_Typedef VT13;

ALL_RX_Data_T ALL_RX;

User_Data_T User_data;


MOTOR_Typdef Motors;

struct mecanumInit_typdef mecanumInit_t;

FuzzyRule_t fuzzyRules;

PID_t temppid;

IMU_Data_t IMU_Data;

V_Input_t V_Input;

STATUS_Typedef ALL_STATUS;

Data_RX_0x302_t Data_RX_0x302;

CONTAL_Typedef ALL_CONTROL;

union COMMUNICATION_UNION_Typdef Communication;

uint8_t state_master;
select_t wuxianchuankou;
uint8_t ReadBuffer_Picture[156];//图传第一次缓冲
Circular_TypeDef Circular_Buffer_Picture;
//***********************************************************************************************************
void Everying_Init(void)
{
	HAL_Delay(1000);
	
	Command_Init(&Circular_Buffer_Picture);
	
	HAL_UART_DMAStop(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
  volatile uint32_t tmp1 = huart1.Instance->RDR;
  (void)tmp1;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReadBuffer_Picture, sizeof(ReadBuffer_Picture));
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
	
	
	HAL_UART_DMAStop(&huart5);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
  volatile uint32_t tmp5 = huart5.Instance->RDR;
  (void)tmp5;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, sizeof(DBUS_RX_DATA));
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
	
	
    //DWT初始化
    DWT_Init();

//    //陀螺仪初始化
//	Fuzzy_Rule_Init(&fuzzyRules, NULL, NULL, NULL, 
//					imu_temp_fuzzy[0], imu_temp_fuzzy[1], imu_temp_fuzzy[2], 
//					0, 0);
//	temppid.FuzzyRule = &fuzzyRules;	
//	PID_Init(&temppid,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_MAX_IOUT,
//				imu_temp_PID,0,0,0.5f,0.5f,2,Integral_Limit);
//    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//    while(BMI088_init()){}
		
			
//	  //ADC初始化	
//	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)V_Input.adc_val,2);
	
    //定时器初始化
	  HAL_TIM_Base_Start_IT(&htim7);
		
			
	  //FDCAN初始化
    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();

			
//    //开启XT30 2+2 可控输出
//    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);


    //开启对外5V
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);


	  //舵机PWM初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);


	  //蜂鸣器PWM初始化
    BSP_Buzzer_Init();
		
//		F_MOTOR_PID_HEAD_INIT(&Motors );

		//机械臂初始化
		Motors.DM_8009_J1.PV.SPE = 2.0;
		Motors.DM_8009_J2.PV.SPE = 2.0;
		Motors.DM_4340_J3.PV.SPE = 2.0;
		Motors.DM_4340_J4.PV.SPE = 2.0;
		Motors.DM_4310_J5.PV.SPE = 2.0;
		Motors.DM_4310_J6.PV.SPE = 2.0;
		
		HAL_Delay(100);
		ALL_CONTROL.Slave.Soft_Start_Chassis = 0.03;
		
//		//50-250
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,50 );//YAW238
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,50);//pITCH120
		
		ALL_CONTROL.Mode = Custom_Pairing;
		ALL_CONTROL.Terminal = Clamp_Close;
		
		
		state_master = 1;
		
}

void StartDefaultTask(void const * argument)
{
	osDelay(50);
	
	MX_USB_DEVICE_Init();
	
//	//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 255);//图传串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)RX_temporary, 255);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);//上位机串口
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)ALL_RX.Data, 510);//裁判系统串口
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90);//遥控串口
	
	for(;;)
	{
		osDelay(1);
	}
}





