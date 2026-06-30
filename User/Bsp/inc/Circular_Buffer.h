#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include <string.h>
#include <stdint.h>

#define BUFFER_SIZE 1248         // 循环缓冲区大小

typedef struct {
	
    uint8_t Pack_Head1;  // 包头1 
    uint8_t Pack_Head2;  // 包头2 如果没有包头2的话，参数给0即可
    uint16_t length;     // 数据包总长度
	
} DataPacket_TypeDef;

typedef struct {
    uint8_t Circular_buffer[BUFFER_SIZE]; // 循环缓冲区（一个串口一个缓冲区）
    uint16_t readIndex;  
    uint16_t writeIndex; 

		DataPacket_TypeDef Custom_Control;//自定义控制器
		DataPacket_TypeDef Grey_Control;//灰控

} Circular_TypeDef;

void Command_Init(Circular_TypeDef* Circular);
void Command_AddReadIndex(Circular_TypeDef *Circular_Buffer, uint16_t length);
uint8_t Command_Read(Circular_TypeDef *Circular_Buffer, uint16_t i);
uint16_t Command_GetLength(Circular_TypeDef *Circular_Buffer);
uint16_t Command_GetRemain(Circular_TypeDef *Circular_Buffer);
uint16_t Command_Write(Circular_TypeDef *Circular_Buffer, uint8_t *data, uint16_t length);
uint16_t Command_GetCommand_Custom(Circular_TypeDef *Circular_Buffer, uint8_t *command);
uint16_t Command_GetCommand_Grey(Circular_TypeDef *Circular_Buffer, uint8_t *command);

#endif /* INC_COMMAND_H_ */