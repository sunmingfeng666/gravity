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

#include "can_bsp.h"

void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000; //
    sFilterConfig.FilterID2 = 0x00000000; //
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* 全局过滤设置 */
    /* 接收到消息ID与标准ID过滤不匹配，不接受 */
    /* 过滤标准ID远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
//    /* 开启RX FIFO0的新数据中断 */
//    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//    {
//        Error_Handler();
//    }
		uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | 
                          FDCAN_IT_BUS_OFF |           // 开启 Bus-Off (离线) 中断 (必须)
                          FDCAN_IT_ERROR_WARNING |     // 开启错误警告中断 (可选，错误计数>96触发)
                          FDCAN_IT_ERROR_PASSIVE;      // 开启被动错误中断 (可选，错误计数>127触发)

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, active_its, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType =  FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* Configure global filter:
    Filter all remote frames with STD and EXT ID
    Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType =  FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* Configure global filter:
    Filter all remote frames with STD and EXT ID
    Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 返回值说明：
 * 0: 发送成功 (成功放入FIFO)
 * 1: FIFO已满，发送失败 (可能是总线故障导致数据发不出去)
 * 2: 硬件添加消息失败
 * 3: 数据长度参数错误
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;                 // CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.MessageMarker = 0;               // 消息标记
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    // 【修改点1】判断是经典CAN还是CANFD
    // 注意：如果是经典CAN，len不能大于8！
    if (len <= 8) {
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN; 
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    } else {
        TxHeader.FDFormat = FDCAN_FD_CAN; // 大于8字节必须用CANFD格式
        // TxHeader.BitRateSwitch = FDCAN_BRS_ON; // 如果开启了波特率切换，这里打开
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF; 
    }

    // 【修改点2】修复数据长度映射问题 (STM32G4/H7等FDCAN需要使用特定的宏)
    switch(len)
    {
        case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
        case 12: TxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
        case 16: TxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
        case 20: TxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
        case 24: TxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
        case 32: TxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
        case 48: TxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
        case 64: TxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
        default: return 3; // 不支持的长度，直接返回错误
    }

    // 【修改点3】发送前检查 FIFO 是否有空闲位置
    // 如果没有空闲位置，说明前面的数据堵死了，直接返回错误，不让程序卡死
    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0)
    {
        // 可选：在这里可以判断是不是发生 Bus-Off 了
        // 如果 FDCAN 处于 Bus-Off 状态，可能需要重启 CAN
        return 1; // 报告 FIFO 满，丢弃该帧或由上层决定是否重试
    }

    // 【修改点4】执行发送，并去掉了 Error_Handler()
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
    {
        return 2; // 添加失败
    }

    return 0; // 发送成功
}




