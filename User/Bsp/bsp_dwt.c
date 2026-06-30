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

/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi (Refactored for STM32H7 High Performance)
 * @version V2.0.0
 * @brief   STM32H7 高精度DWT时间测量与延时 (防溢出、高效率重构版)
 ******************************************************************************
 */
#include "bsp_dwt.h"
#include "stm32h7xx_hal.h" // 依赖HAL库获取 SystemCoreClock

/* 全局时间结构体 (确保在 bsp_dwt.h 中定义了 DWT_Time_t) */
DWT_Time_t SysTime;

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;

static void DWT_CNT_Update(void);

/**
 * @brief  初始化DWT（无参数，自动获取主频并解锁LAR）
 */
void DWT_Init(void)
{
    /* 1. 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 2. STM32H7 (Cortex-M7) 必须解锁DWT LAR寄存器 */
    DWT->LAR = 0xC5ACCE55; 

    /* 3. DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = 0;

    /* 4. 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 5. 自动获取当前CPU频率 */
    CPU_FREQ_Hz = SystemCoreClock; 
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    /* 6. 初始化时间戳 */
    CYCCNT_LAST = DWT->CYCCNT;
    CYCCNT64 = 0;
}

/**
 * @brief  核心：更新64位全局时间戳（无符号减法完美解决溢出漏帧）
 */
static void DWT_CNT_Update(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    // 利用32位无符号减法特性，天然处理了溢出情况，无需繁杂的if判断
    CYCCNT64 += (cnt_now - CYCCNT_LAST); 
    CYCCNT_LAST = cnt_now;
}

/**
 * @brief  更新系统结构体时间 (s, ms, us) 
 * @note   优化了数学运算，直接通过求余获得精确分割，消除了浮点累加的精度丢失
 */
void DWT_SysTimeUpdate(void)
{
    DWT_CNT_Update();

    // 更新 SysTime 结构体 (避免多次庞大的64位除法，改用求余计算)
    SysTime.s  = CYCCNT64 / CPU_FREQ_Hz;
    SysTime.ms = (CYCCNT64 % CPU_FREQ_Hz) / CPU_FREQ_Hz_ms;
    SysTime.us = (CYCCNT64 % CPU_FREQ_Hz_ms) / CPU_FREQ_Hz_us;
}

/**
 * @brief  获取与上一次调用相比的增量时间 (单精度浮点，秒)
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    // 无符号减法天然防溢出
    uint32_t diff = cnt_now - *cnt_last; 
    *cnt_last = cnt_now;

    DWT_CNT_Update(); // 同步维持64位计数器运转

    return (float)diff / (float)CPU_FREQ_Hz;
}

/**
 * @brief  获取与上一次调用相比的增量时间 (双精度浮点，秒)
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    uint32_t diff = cnt_now - *cnt_last;
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return (double)diff / (double)CPU_FREQ_Hz;
}

/**
 * @brief  获取系统运行绝对时间 (秒)
 * @note   注意：返回float在运行几小时后，微秒级的精度会被float有效位抛弃
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    // 摒弃了原代码中 SysTime.s + ms*0.001f 的阶梯型拼接法，直接从最精确的底座除出来
    return (float)CYCCNT64 / (float)CPU_FREQ_Hz;
}

/**
 * @brief  获取系统运行绝对时间 (毫秒)
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return (float)CYCCNT64 / (float)CPU_FREQ_Hz_ms;
}

/**
 * @brief  获取系统运行绝对时间 (微秒，64位永不溢出)
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return CYCCNT64 / CPU_FREQ_Hz_us;
}

/**
 * @brief  秒级浮点延时
 * @param  Delay: 延时的秒数 (如 0.001f 代表 1ms)
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t delay_ticks = (uint32_t)(Delay * (float)CPU_FREQ_Hz);

    // 最大支持约 7.8 秒 (在 550MHz 主频下)
    while ((DWT->CYCCNT - tickstart) < delay_ticks);
}

/**
 * @brief  微秒级阻塞延时 (去除了原代码的 10000us 上限，化繁为简)
 * @param  uSec: 延时的微秒数 (uint16_t 最大支持 65535us 即 65.5ms)
 */
void DWT_delay_us(uint16_t uSec)
{
    uint32_t ticks_start = DWT->CYCCNT;
    uint32_t ticks_delay = (uint32_t)uSec * CPU_FREQ_Hz_us;

    // 利用无符号减法，天然免疫单次溢出，彻底抛弃原先臃肿的 if-else 溢出判断
    while ((DWT->CYCCNT - ticks_start) < ticks_delay);
}