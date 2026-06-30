/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                               神兽保佑            永无BUG
 */
#include "Monitor_Task.h"
#include "All_Init.h"


/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //ROOT函数
 *	@parameter:		    //
 *	@time:				//23-04-13 17:38
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void RUI_F_ROOT(STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typdef *MOTOR)
{
    RUI_F_ROOT_ALL_STATUS(Root, DBUS, RX_0x302, MOTOR);
    RUI_F_MASTER_LOCATION_JUDGE(Root);
    RUI_F_LED_SHOW_STATUS(Root);
}

/************************************************************万能分隔符**************************************************************
// * 	@author:			//WHW
// *	@performance:	    //大疆电机离线检测
// *	@parameter:		    //
// *	@time:				//23-04-13 15:01
// *	@ReadMe:			//
// ************************************************************万能分隔符**************************************************************/
uint8_t DJI_F_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return RUI_DF_OFFLINE;
    }
    else
        return RUI_DF_ONLINE;
}

/************************************************************万能分隔符**************************************************************
// *	@performance:	    //达妙电机离线检测
// *	@parameter:		    //
// *	@time:				//23-04-13 15:01
// *	@ReadMe:			//
************************************************************万能分隔符**************************************************************/
uint8_t DM_F_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
{
  DATA->ONLINE_JUDGE_TIME--;

  if (DATA->ONLINE_JUDGE_TIME < 5 || DATA->state == 0x08||  DATA->state == 0x09 || DATA->state == 0x0A || DATA->state == 0x0B || DATA->state == 0x0C || DATA->state == 0x0D || DATA->state == 0x0E)
  {
    DATA->ONLINE_JUDGE_TIME = 0;
    return RUI_DF_OFFLINE;
  }else
	{
		return RUI_DF_ONLINE;
	}
    
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //总的离线检测
 *	@parameter:		    //
 *	@time:				//23-03-29 19:34
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-13 15:22    bestrui
 *  @UpData：           //电机检测封装成函数
 ************************************************************万能分隔符**************************************************************/
void RUI_F_ROOT_ALL_STATUS(STATUS_Typedef *Root, DBUS_Typedef *DBUS, Data_RX_0x302_t *RX_0x302, MOTOR_Typdef *MOTOR)
{
    //遥控在线监测
    if (DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME < 5)
    {
        DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME = 3;
        Root->RM_DBUS = RUI_DF_OFFLINE;
    }
    else
    {
        Root->RM_DBUS = RUI_DF_ONLINE;
    }
    DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME--;

    //自定义控制器在线监测
    if (VT13.ONLINE_JUDGE_TIME < 5)
    {
        VT13.ONLINE_JUDGE_TIME = 3;
        Root->RM_Image = RUI_DF_OFFLINE;
    }
    else
    {
        Root->RM_Image = RUI_DF_ONLINE;
    }
    VT13.ONLINE_JUDGE_TIME--;

    //电机在线检测
		Root->STATUS_ARM_8009_1 =        DM_F_MOTOR_STATUS(&MOTOR->DM_8009_J1.DATA);
		Root->STATUS_ARM_8009_2 =        DM_F_MOTOR_STATUS(&MOTOR->DM_8009_J2.DATA);
		Root->STATUS_ARM_4340_ROLL =     DM_F_MOTOR_STATUS(&MOTOR->DM_4340_J3.DATA);
		Root->STATUS_ARM_4340_PITCH_1 =  DM_F_MOTOR_STATUS(&MOTOR->DM_4340_J4.DATA);
		Root->STATUS_ARM_4310_PITCH_2 =  DM_F_MOTOR_STATUS(&MOTOR->DM_4310_J5.DATA);
		Root->STATUS_ARM_4310_ROLL =     DM_F_MOTOR_STATUS(&MOTOR->DM_4310_J6.DATA);
		Root->STATUS_ARM_3507_TERMINAL = DM_F_MOTOR_STATUS(&MOTOR->DM_3507_Terminal.DATA);
    


		//fdcan总线检测
		Root->STATUS_FDCAN_1 = Monitor_FDCAN(&hfdcan1);
		Root->STATUS_FDCAN_2 = Monitor_FDCAN(&hfdcan2);
		Root->STATUS_FDCAN_3 = Monitor_FDCAN(&hfdcan3);

}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //主控位置判断
 *	@parameter:		    //
 *	@time:				//23-03-29 19:34
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t RUI_F_MASTER_LOCATION_JUDGE(STATUS_Typedef *Root)
{
    Root->MASTER_LOCATION = RUI_DF_MASTER_LOCATION_HEAD;
    return RUI_DF_READY;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //用LED灯显示状态
 *	@parameter:		    //
 *	@time:				//23-04-13 17:36
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:03    bestrui
 *  @UpData：           //底盘正常改为绿灯闪烁
 ************************************************************万能分隔符**************************************************************/
void RUI_F_LED_SHOW_STATUS(STATUS_Typedef *Root)
{


//		if(Root->RM_DBUS == RUI_DF_OFFLINE){
//				WS2812_Ctrl(255,255,0);//黄色常量
//				return;
//		}
		if(Root->RM_Image == RUI_DF_OFFLINE){
			WS2812_Ctrl(255,80,0);//橙色常量
			return;
		}

	  if(Root->STATUS_ARM_3507_TERMINAL == 0 ||
			 Root->STATUS_ARM_4310_PITCH_2 == 0  ||
			 Root->STATUS_ARM_4310_ROLL == 0 		 ||
			 Root->STATUS_ARM_4340_PITCH_1 ==0   ||
			 Root->STATUS_FDCAN_3 != 0){
		    RGB_Ctrl_Count(0 ,3);
		    return;
	   }
		if(Root->STATUS_ARM_4340_ROLL == 0  ||
			 Root->STATUS_ARM_8009_1 == 0 	 	||
			 Root->STATUS_ARM_8009_2 ==0      ||
			 Root->STATUS_FDCAN_2 != 0){
				RGB_Ctrl_Count(0 ,2);
				return;
		 }
			//一切正常绿色常量
	WS2812_Ctrl(0,255,0);

}

/**----------------------------------------
 * 	@author:			    sheep
 *	@brif:	    监测fdcan总线
 *	@parameter:		    
 *	@ReadMe:		待测试
 *----------------------------------------
 */
uint8_t Monitor_FDCAN(FDCAN_HandleTypeDef *hcan)
{
	uint32_t psr_value = hcan->Instance->PSR;
	
	if(psr_value & (1U << 7)){
		return 1;//处于Bus_Off;
	}
	
	return 0;
}


