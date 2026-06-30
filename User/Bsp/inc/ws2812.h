#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h" 
#include "bsp_dwt.h"

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void RGB_Ctrl(uint8_t COLOR, float TIME, uint8_t Power);
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void RGB_Ctrl_Count(uint8_t COLOR , int count);
void RGB_Yellow_Orange_Alternate(void);

#endif
