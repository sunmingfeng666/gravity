/********************************************************************************************** 
*   Author : Keysking
*   Mover +  : Sheep
**********************************************************************************************/ 

#include "Circular_Buffer.h"
#include "CRC_DJI.h"
#include "VT13.h"
#include <stdbool.h>

/**
* @brief 初始化循环缓冲区
*/
void Command_Init(Circular_TypeDef* Circular) {
	//自定义控制器数据包
    Circular->Custom_Control.Pack_Head1 = 0xA5;
    Circular->Custom_Control.Pack_Head2 = 0x00;
    Circular->Custom_Control.length = 39;
	//灰控数据包
		Circular->Grey_Control.Pack_Head1 = 0xA9;
    Circular->Grey_Control.Pack_Head2 = 0x53;
    Circular->Grey_Control.length = 21;
	//缓冲区指针
    Circular->readIndex = 0;
    Circular->writeIndex = 0;
}

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(Circular_TypeDef *Circular_Buffer, uint16_t length) { 
    Circular_Buffer->readIndex += length;
    Circular_Buffer->readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param *Circular_Buffer 循环缓冲区结构体指针
* @param i 要读取的数据索引
*/
uint8_t Command_Read(Circular_TypeDef *Circular_Buffer, uint16_t i) {
    uint16_t index = i % BUFFER_SIZE;
    return Circular_Buffer->Circular_buffer[index];
}

/**
* @brief 计算缓冲区中已使用的数据长度
* @return 返回缓冲区中已使用的数据长度
*/
uint16_t Command_GetLength(Circular_TypeDef *Circular_Buffer) {
    // 读索引等于写索引时，缓冲区为空
    if (Circular_Buffer->readIndex == Circular_Buffer->writeIndex) {
        return 0;
    }
    // 如果缓冲区未满,返回未处理的数据长度
    if (Circular_Buffer->readIndex < Circular_Buffer->writeIndex) {
        return Circular_Buffer->writeIndex - Circular_Buffer->readIndex;
    } else {
        return BUFFER_SIZE - Circular_Buffer->readIndex + Circular_Buffer->writeIndex;
    }
}

/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
*/
uint16_t Command_GetRemain(Circular_TypeDef *Circular_Buffer) {
    return BUFFER_SIZE - Command_GetLength(Circular_Buffer);
}

/**
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @readme 在中断回调函数中调用,length为HAL_UARTEx_RxEventCallback的Size
* @return 写入的数据长度
*/
uint16_t Command_Write(Circular_TypeDef *Circular_Buffer, uint8_t *data, uint16_t length) {
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain(Circular_Buffer) < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (Circular_Buffer->writeIndex + length <= BUFFER_SIZE) {
        memcpy(Circular_Buffer->Circular_buffer + Circular_Buffer->writeIndex, data, length);
        Circular_Buffer->writeIndex += length;
        if(Circular_Buffer->writeIndex == BUFFER_SIZE) { // 越界归零保护
            Circular_Buffer->writeIndex = 0;
        }
    } else {
        uint16_t firstLength = BUFFER_SIZE - Circular_Buffer->writeIndex;
        memcpy(Circular_Buffer->Circular_buffer + Circular_Buffer->writeIndex, data, firstLength);
        memcpy(Circular_Buffer->Circular_buffer, data + firstLength, length - firstLength);
        Circular_Buffer->writeIndex = length - firstLength;
    }
    return length;
}

uint16_t Command_GetCommand_Custom(Circular_TypeDef *Circular_Buffer, uint8_t *command) {
    uint16_t available_len;
    // 协议常数定义
    const uint8_t FrameHeader_Length = 5;
    const uint8_t CMDID_Length = 2;
    const uint8_t CRC16_Length = 2;
    // 最小有效帧长度 = 包头(5) + 命令码(2) + 帧尾(2) = 9
    const uint8_t MIN_FRAME_LEN = FrameHeader_Length + CMDID_Length + CRC16_Length; 

    // 只要缓冲区里的数据大于等于最小帧长度，就持续寻找
    while ((available_len = Command_GetLength(Circular_Buffer)) >= MIN_FRAME_LEN) {
        
        // 1. 寻找起始符 SOF (0xA5)
        if (Command_Read(Circular_Buffer, Circular_Buffer->readIndex) != 0xA5) {
            Command_AddReadIndex(Circular_Buffer, 1);
            continue; 
        }

        // 2. 提取 5 字节帧头到连续数组，用于 CRC8 校验和解析长度
        uint8_t header[5];
        for (uint8_t i = 0; i < FrameHeader_Length; i++) {
            header[i] = Command_Read(Circular_Buffer, Circular_Buffer->readIndex + i);
        }

        // 3. CRC8 校验帧头
        if (Verify_CRC8_Check_Sum(header, FrameHeader_Length) == 0) {
            Command_AddReadIndex(Circular_Buffer, 1); 
            continue;
        }

        // 4. 解析数据段长度 (小端模式)
        uint16_t payload_length = (uint16_t)(header[2] << 8 | header[1]);
        
        // 计算整帧总长度
        uint16_t total_length = payload_length + MIN_FRAME_LEN;

        // 防止异常数据引发的内存溢出（指令最大长度通常不会超过缓冲区）
        if(total_length > BUFFER_SIZE) {
            Command_AddReadIndex(Circular_Buffer, 1);
            continue;
        }

        // 5. 检查环形缓冲区中，是否已经接收到了完整的这帧数据
        if (available_len < total_length) {
            return 0; 
        }

        // 6. 将整帧完整数据提取到连续的 command 数组中
        for (uint16_t i = 0; i < total_length; i++) {
            command[i] = Command_Read(Circular_Buffer, Circular_Buffer->readIndex + i);
        }

        // 7. CRC16 校验整帧
        if (Verify_CRC16_Check_Sum(command, total_length) == 0) {
            Command_AddReadIndex(Circular_Buffer, 1); 
            continue;
        }

        // 8. 至此，CRC8、长度、CRC16 均合法
        Command_AddReadIndex(Circular_Buffer, total_length);
        
        // 返回该帧的总长度，告知外部有一条有效指令
        return total_length; 
    }

    return 0;
}
    uint16_t available_len;
uint16_t Command_GetCommand_Grey(Circular_TypeDef *Circular_Buffer, uint8_t *command) {

    // VT13协议常数定义
    const uint8_t VT13_FrameHeader1 = 0xA9;
    const uint8_t VT13_FrameHeader2 = 0x53;
    const uint16_t VT13_FrameLength = 21; // 固定帧长度21字节
    const uint8_t CRC16_Length = 2;

    // 只要缓冲区里的数据大于等于VT13帧长度，就持续寻找
    while ((available_len = Command_GetLength(Circular_Buffer)) >= VT13_FrameLength) {
        
        // 1. 寻找帧头1 (0xA9)
        if (Command_Read(Circular_Buffer, Circular_Buffer->readIndex) != VT13_FrameHeader1) {
            Command_AddReadIndex(Circular_Buffer, 1);
            continue; 
        }

        // 2. 检查帧头2 (0x53)
        if (Command_Read(Circular_Buffer, Circular_Buffer->readIndex + 1) != VT13_FrameHeader2) {
            Command_AddReadIndex(Circular_Buffer, 1);
            continue;
        }

        // 3. 检查环形缓冲区中，是否已经接收到了完整的21字节帧
        if (available_len < VT13_FrameLength) {
            return 0; 
        }

        // 4. 将整帧完整数据提取到连续的 command 数组中
        for (uint16_t i = 0; i < VT13_FrameLength; i++) {
            command[i] = Command_Read(Circular_Buffer, Circular_Buffer->readIndex + i);
        }

        // 5. CRC16 校验整帧 (使用标准CRC-16/CCITT-FALSE)
        // 注意：CRC校验范围是前19字节(21-2)
				
				if(verify_crc16_check_sum(command,21) == 0){
            Command_AddReadIndex(Circular_Buffer, 1); 
            continue;
        }

        // 6. 至此，帧头、长度、CRC16 均合法
        Command_AddReadIndex(Circular_Buffer, VT13_FrameLength);
        
        // 返回该帧的总长度，告知外部有一条有效指令
        return VT13_FrameLength; 
    }

    return 0;
}