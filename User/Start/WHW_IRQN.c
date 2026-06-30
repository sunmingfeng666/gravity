/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "All_Init.h"
#include "Usart_Task.h"
#include "RUI_MATH.h"
#include "One_Click_Task.h"
#include "HID_Control.h"
#include "DM_test.h"



uint8_t move;
float a = 1.0f;
float t1,t2,dt;
static uint8_t TX[12] = {0x3A,0x98,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};
UBaseType_t stack_free = 0;

uint8_t commandLength_Custom = 0;
uint8_t commandLength_Grey = 0;
uint8_t aaa[21] = {0};

//************************************************************************************************************
//画UI与平时测试
int i =0;
void StartRobotUITask(void const *argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();
	
    for(;;)
    {

			i++;
			
			
			DWT_SysTimeUpdate(); //必要的一步
			if(state_master ==1)
			{
				Send_To_Slave_Machine();
			}
			
			
        vTaskDelayUntil(&currentTimeRobotUI, 10);
    }
}

//************************************************************************************************************
void StartIMUTask(void const *argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();
	
    for(;;)
    {
        t1 = DWT_GetTimeline_ms();
        //INS_Task(&IMU_Data, &temppid);
			

			if(ALL_CONTROL.One_Click_Step ==0 && ALL_CONTROL.One_Click_Store == 0 && ALL_CONTROL.One_Click_take == 0 && ALL_CONTROL.One_Click_Reset ==0 )
			{
				switch(ALL_CONTROL.Mode)
				{
					case Pose_Take://取位姿
						if(ALL_CONTROL.LastMode == Normal)  
						{
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J1_8009p = -1.6760f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = 1.0008f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J3_4340 =  1.5078f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = 1.7660f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = 1.1847f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J6_4310 = -0.8436f;				
							
							ALL_CONTROL.ModeChange_Normal_To_Pose_Take = 1;
								
						}else if(ALL_CONTROL.LastMode == Custom_Control )
						{
							
							ALL_CONTROL.control[1].MECHANICAL_ARM.J1_8009p = -1.6760f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = 1.0008f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J3_4340 =  1.5078f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = 1.7660f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = 1.1847f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J6_4310 = -0.8436f;
            
						}else if(ALL_CONTROL.LastMode == Selt_Prosurant)
						{
							ALL_CONTROL.One_Click_Self_Prosurant = 2;
						}

						ALL_CONTROL.control[1].PICTURE.Picture_Lift = 1000000;
						htim2.Instance->CCR1 = 180;
						htim2.Instance->CCR3 = 230;
						ALL_CONTROL.LastMode = Pose_Take; 
						break;
        
					case Custom_Control://自定义控制器
					
						if(ALL_CONTROL.One_Click_Store == 0 && ALL_CONTROL.One_Click_take == 0)
						{
							F_RX302_CONTROL(&Data_RX_0x302, &ALL_CONTROL);
						}
						
						ALL_CONTROL.control[1].PICTURE.Picture_Lift = 1000000;
						htim2.Instance->CCR1 = 180;
						htim2.Instance->CCR3 = 230;
						ALL_CONTROL.LastMode = Custom_Control; // 跑完之后更新状态
						break;
						
					case Normal://正常开车姿态
        
						if(ALL_CONTROL.LastMode == Custom_Pairing) 	
						{
							ALL_CONTROL.control[1].MECHANICAL_ARM.J1_8009p = -0.8310f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = -0.0753f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J3_4340 = 0.0811f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = 1.8637f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = -0.8379f;
							ALL_CONTROL.control[1].MECHANICAL_ARM.J6_4310 =  -2.6858f;
	
							
							
						}
						else if(ALL_CONTROL.LastMode == Pose_Take)  
						{
							
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J1_8009p = -0.8310f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J2_8009P = -0.0753f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J3_4340 = 0.0811f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J4_4340 = 1.8637f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J5_4310 = -0.8379f;
//							ALL_CONTROL.control[1].MECHANICAL_ARM.J6_4310 =  -2.6858f;
							
							ALL_CONTROL.ModeChange_Pose_Take_To_Normal = 1;
							
							
							
						}
						ALL_CONTROL.LastMode = Normal; 
						
						ALL_CONTROL.control[1].PICTURE.Picture_Lift = 0;
						htim2.Instance->CCR1 = 75;//YAW
						htim2.Instance->CCR3 = 230;//PITCH
						//ALL_CONTROL.control[2].PICTURE.Picture_Transverse = -600000;//（开车时的横移位置）
						break;
        
					case Custom_Pairing:
						ALL_CONTROL.LastMode = Custom_Pairing; // 离开前，记录自己
						ALL_CONTROL.Mode = Normal;             // 切换到下一个状态
						break;
					case Selt_Prosurant:
					
						if(ALL_CONTROL.LastMode == Pose_Take)
						{
							ALL_CONTROL.One_Click_Self_Prosurant = 1;
						}
					
						ALL_CONTROL.control[1].PICTURE.Picture_Lift = 0;
						htim2.Instance->CCR1 = 75;//YAW
						htim2.Instance->CCR3 = 230;//PITCH
						
						ALL_CONTROL.LastMode = Selt_Prosurant; 
						break;
					}
			}
						
      t2 = DWT_GetTimeline_ms();
      dt = (t2 - t1);
      vTaskDelayUntil(&currentTimeIMU, 2);
    }
} 

//************************************************************************************************************
//运动
void StartGimbalTask(void const *argument)
{
    portTickType currentTimeGimbal;
    currentTimeGimbal = xTaskGetTickCount();

    for(;;)
    {
      osDelay(5);		
			
			//从机相关电机的缓启动
			HID_Control(VT13, &ALL_CONTROL);
			if (J2ID_IsolationMode == 0U)
			{
				Motor_Soft_Start();
        Motor_MoveControl ();
			}

			
			//检测堆栈的最大值
			stack_free = uxTaskGetStackHighWaterMark (NULL);
			
			
    }
}

//************************************************************************************************************
//状态监控
void StartMonitorTask(void const * argument)
{
    portTickType currentTimeMonitor;
    currentTimeMonitor = xTaskGetTickCount();

    for(;;)
    {
			
      RUI_F_ROOT(&ALL_STATUS, &WHW_V_DBUS, &Data_RX_0x302, &Motors);

			osDelay(5);
    }
}

//************************************************************************************************************
//系统辨识
void StartK3debugTask(void const * argument)
{
    portTickType currentTimeK3debug;
    currentTimeK3debug = xTaskGetTickCount();

	
    for(;;)
    {

			commandLength_Grey = Command_GetCommand_Grey(&Circular_Buffer_Picture,aaa);
		  if(commandLength_Grey != 0)
			{
				VT13_Resovled(aaa,&VT13);
				VT13_CONTROL(VT13,&ALL_CONTROL);
			}
			if(ALL_CONTROL.Mode == Custom_Control)
			{
				commandLength_Custom = Command_GetCommand_Custom(&Circular_Buffer_Picture,Data_RX_0x302.Data);		
			}
			
			osDelay(5);
    }
}

//************************************************************************************************************
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader1;
    uint8_t g_Can1RxData[64];
	

    FDCAN_RxHeaderTypeDef RxHeader3;
    uint8_t g_Can3RxData[64];  
	
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)//5M波特率
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            switch(RxHeader1.Identifier)
            {
							//测试
                case 0x14:
                    dmxx_fbdata(&Motors.ceshiDM_4310.DATA, g_Can1RxData,RxHeader1.DataLength );
                    break;
								
								//机械臂
                case 0x17://J4
                    dmxx_fbdata(&Motors.DM_4340_J4.DATA, g_Can1RxData,RxHeader1.DataLength );
                    break;
                case 0x18://J5
                    dmxx_fbdata(&Motors.DM_4310_J5.DATA, g_Can1RxData,RxHeader1.DataLength );
                    break;
                case 0x19://J6
                    dmxx_fbdata(&Motors.DM_4310_J6.DATA, g_Can1RxData,RxHeader1.DataLength );
                    break;
                case 0x1A://末端
                    dmxx_fbdata(&Motors.DM_3507_Terminal.DATA, g_Can1RxData,RxHeader1.DataLength );
                    break;
            }
        }
		

        if(hfdcan->Instance == FDCAN3)
        {

            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
							switch(RxHeader3.Identifier)
            {
							//测试
                case 0x14:
                    dmxx_fbdata(&Motors.ceshiDM_4310.DATA, g_Can3RxData,RxHeader3.DataLength );
                    break;
								
								//机械臂
                case 0x17://J4
                    dmxx_fbdata(&Motors.DM_4340_J4.DATA, g_Can3RxData,RxHeader3.DataLength );
                    break;
                case 0x18://J5
                    dmxx_fbdata(&Motors.DM_4310_J5.DATA, g_Can3RxData,RxHeader3.DataLength );
                    break;
                case 0x19://J6
                    dmxx_fbdata(&Motors.DM_4310_J6.DATA, g_Can3RxData,RxHeader3.DataLength );
                    break;
                case 0x1A://末端
                    dmxx_fbdata(&Motors.DM_3507_Terminal.DATA, g_Can3RxData,RxHeader3.DataLength );
                    break;
            }
	       }
			 }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader2;
    uint8_t g_Can2RxData[64];
	
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if(hfdcan->Instance == FDCAN2)//1M波特率
        {
            /* Retrieve Rx messages from RX FIFO1 */
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
            switch(RxHeader2.Identifier)
            {
                case 0x14://J1
                    dmxx_fbdata(&Motors.DM_8009_J1.DATA, g_Can2RxData,RxHeader2.DataLength );
                    break;
                case 0x15://J2
                    dmxx_fbdata(&Motors.DM_8009_J2.DATA, g_Can2RxData,RxHeader2.DataLength );
                    break;
                case 0x16://J3
                    dmxx_fbdata(&Motors.DM_4340_J3.DATA, g_Can2RxData,RxHeader2.DataLength );
                    break;
            }
					}
	}
}
//************************************************************************************************************

//清除can总线关闭
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 判断是哪个 FDCAN 触发的 (如果你有多个 CAN)
    if (hfdcan->Instance == FDCAN1)
    {
        // 检查是不是 Bus-Off 导致的中断
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
        {
            // 注意：千万别在这里用 HAL_Delay() 长延时！中断里延时会卡死系统
            
            // 执行重启操作，清除 INIT 位，让它开始监听 128x11 个隐性位
            HAL_FDCAN_Stop(hfdcan);  
            HAL_FDCAN_Start(hfdcan); 
            
            // ★非常重要★：Stop 操作会清除掉之前开启的中断配置，所以必须重新开启一次！
            uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | 
																	FDCAN_IT_BUS_OFF |           // 开启 Bus-Off (离线) 中断 (必须)
																	FDCAN_IT_ERROR_WARNING |     // 开启错误警告中断 (可选，错误计数>96触发)
																	FDCAN_IT_ERROR_PASSIVE;      // 开启被动错误中断 (可选，错误计数>127触发)
            HAL_FDCAN_ActivateNotification(hfdcan, active_its, 0);
            
        }
    }
}

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
#define  BUFFER_SIZE_2  (18)
#define  BUFFER_SIZE_3  (29)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		if (Size <= 18)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收完毕后重启
			RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);
			F_DUBS_CONTROL(&WHW_V_DBUS ,&ALL_CONTROL);
			
			//更新在线时间
			WHW_V_DBUS.ONLINE_JUDGE_TIME = 30;
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收完毕后重启
			memset(DBUS_RX_DATA, 0, 18);							   
		}
	}
//------------------------------------------------------------------------------------
	if(huart->Instance == USART10)//主从通讯串口
	{
		
		memset(Communication.Data, 0, sizeof(Communication.Data));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, Communication.Data, sizeof(Communication.Data));
	}
//------------------------------------------------------------------------------------	
	if(huart->Instance == USART1)//图传链路
	{
		
		Command_Write(&Circular_Buffer_Picture ,ReadBuffer_Picture, Size);
		
		VT13.ONLINE_JUDGE_TIME = 30;
		
		__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);

		
	}
//------------------------------------------------------------------------------------
	if(huart->Instance == UART7)//无线串口
  {
//		uint8_t data_length_7;
//    data_length_7  = BUFFER_SIZE_3*4 - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);//计算接收到的数据长度
//		
//		Read_Data(&wuxianchuankou , &Data_RX_0x302);
//		Read_Data_second(&wuxianchuankou , &Data_RX_0x302 );//将原始数据分给0x302或0x304
		if(wuxianchuankou.Data[5] == 0x02)//CRC位
       F_RX302_CONTROL(&Data_RX_0x302, &ALL_CONTROL);
		
			
		
		memset((uint8_t*)wuxianchuankou.Data,0,255);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)wuxianchuankou.Data, 255);
		__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	}
}

//************************************************************************************************************
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	
	//遥控器
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收发生错误后重启
		WHW_V_DBUS .ONLINE_JUDGE_TIME = RUI_DF_DBUS_OFFLINE_TIME;
		memset(DBUS_RX_DATA, 0, 18);							   // 清除接收缓存		
	}
	//----------------------------------------------------------------------------------------------------
	if(huart->Instance == USART10)//主控通讯
	{
		
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, Communication.Data, sizeof(Communication.Data));
	}
	//----------------------------------------------------------------------------------------------------
	if(huart->Instance == USART1)//图传串口,获取遥控数据以及键鼠数据
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReadBuffer_Picture, sizeof(ReadBuffer_Picture));
	}
	//----------------------------------------------------------------------------------------------------
  if(huart->Instance == UART7)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)wuxianchuankou.Data, 255);
		memset((uint8_t*)wuxianchuankou.Data, 0, BUFFER_SIZE_3);
  }
}

//************************************************************************************************************
void USER_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM7 )
	{
		
	}
}

//************************************************************************************************************
