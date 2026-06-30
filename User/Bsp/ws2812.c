#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0Ã‚Ã«
#define WS2812_HighLevel   0xF0     // 1Ã‚Ã«

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

void V0_V1(int8_t *tmp)
{
    if(*tmp != 0)   *tmp = 1;
    if(*tmp == 0)   *tmp = 0;
}

void RGB_Ctrl(uint8_t COLOR, float TIME, uint8_t Power)
{
    float TIME_NOW = DWT_GetTimeline_ms();
    static float TIME_LAST = 0;

    static int8_t BIT = 0;

    static uint8_t LED_CONFIG[7][3] ={
        {1, 0, 0},    // RED
        {0, 1, 0},    // GREEN
        {0, 0, 1},    // BLUE
        {1, 1, 0},    // YELLOW
        {1, 0, 1},    // PURPLE
        {0, 1, 1},    // CYAN
        {1, 1, 1},    // write
    };
	
    if ((TIME_NOW - TIME_LAST) > TIME)
    {
        TIME_LAST = TIME_NOW;
		BIT = !BIT;
        V0_V1(&BIT);
    }

    WS2812_Ctrl(LED_CONFIG[COLOR][0]*Power*BIT,
                LED_CONFIG[COLOR][1]*Power*BIT,
                LED_CONFIG[COLOR][2]*Power*BIT);
}

/*
 * 	@author:		Sheep
 *	@performance:	 
 *	@parameter:		 
 *	@ReadMe:  灯闪烁，有明显间隔，目前最多到三（可以扩展功能）
*/
void RGB_Ctrl_Count(uint8_t COLOR , int count)
{
  float TIME_NOW = DWT_GetTimeline_ms();
	static uint32_t start_time = 0;
  static uint8_t active = 0;
	static float TIME_LAST;
	static int8_t led_on = 0;
	
	//---------------------------
	if(active==0)
	{
		start_time = TIME_NOW;
		active = 1;
	}
	
	uint32_t elapsed = TIME_NOW - start_time;
	
	if(elapsed < 200*count)
	{
		if((elapsed - TIME_LAST) > 100)
		{
			TIME_LAST = elapsed;
			led_on = !led_on;	
		}

	}
	else{
		led_on = 0;
		if(elapsed > 1000)
		{
			active = 0;
			TIME_LAST = 0;
		}
	}
	
	
	uint8_t LED_CONFIG[7][3] ={
        {1, 0, 0},    // RED
        {0, 1, 0},    // GREEN
        {0, 0, 1},    // BLUE
        {1, 1, 0},    // YELLOW
        {1, 0, 1},    // PURPLE
        {0, 1, 1},    // CYAN
        {1, 1, 1},    // WRITE
    };
	
		WS2812_Ctrl(LED_CONFIG[COLOR][0]*255*led_on,
                LED_CONFIG[COLOR][1]*255*led_on,
                LED_CONFIG[COLOR][2]*255*led_on);
}
/*
 *  @author:        Sheep
 *  @performance:   黄色与橙色交替闪烁，各占0.25秒
 *  @parameter:     无参数（可根据需要添加亮度参数）
 *  @ReadMe:        黄色(255,255,0)与橙色(255,80,0)交替闪烁
 */
void RGB_Yellow_Orange_Alternate(void)
{
    float TIME_NOW = DWT_GetTimeline_ms();
    static float TIME_LAST = 0;
    static uint8_t color_state = 0; // 0:黄色, 1:橙色
    
    // 每250ms切换一次颜色
    if ((TIME_NOW - TIME_LAST) > 250.0f)
    {
        TIME_LAST = TIME_NOW;
        color_state = !color_state; // 切换颜色状态
        
        // 确保color_state只有0和1两种状态
        if(color_state != 0) color_state = 1;
        if(color_state == 0) color_state = 0;
    }
    
    // 根据当前颜色状态设置LED颜色
    if (color_state == 0) {
        // 黄色 (255, 255, 0)
        WS2812_Ctrl(255, 255, 0);
    } else {
        // 橙色 (255, 80, 0)
        WS2812_Ctrl(255, 80, 0);
    }
}
