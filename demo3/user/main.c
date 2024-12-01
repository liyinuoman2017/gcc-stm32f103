/**
*********************************************************************************************************
*                                      (c) Copyright 2024-2032
*                                         All Rights Reserved
* @By      : liwei
*********************************************************************************************************
**/
typedef unsigned int  uint32_t;
typedef unsigned short  uint16_t;
// 定义STM32F103的寄存器基地址
#define STM32_RCC_BASE          0x40021000   
#define STM32_GPIOB_BASE        0x40010C00   

// 定义寄存器偏移
#define RCC_APB2ENR_OFFSET      0x18   
#define GPIOB_CRL_OFFSET        0x00  
#define GPIOB_CRH_OFFSET        0x04  
#define GPIOB_ODR_OFFSET        0x0C  

// 定义寄存器地址
#define RCC_APB2ENR_REG         (*(volatile uint32_t *)(STM32_RCC_BASE + RCC_APB2ENR_OFFSET))
#define GPIOB_CRL_REG           (*(volatile uint32_t *)(STM32_GPIOB_BASE + GPIOB_CRH_OFFSET))
#define GPIOB_ODR_REG           (*(volatile uint16_t *)(STM32_GPIOB_BASE + GPIOB_ODR_OFFSET))

// 定义位掩码
#define RCC_APB2ENR_IOPBEN      ((uint32_t)0x00000008)  
#define GPIO_MODE_OUT_50MHZ     ((uint32_t)0x0002)      
#define GPIO_CNF_PP             ((uint32_t)0x0000)      
#define GPIO_PIN15              ((uint16_t)0x8000)     
/***********************************************************************************************************
* @描述	: 延时函数
***********************************************************************************************************/
void delay(volatile uint32_t count) 
{
    while (count--) 
	{
        // 空循环
    }
}
/***********************************************************************************************************
* @描述	: 配置GPIOB引脚
***********************************************************************************************************/
void GPIOB_ConfigPin15AsOutput(void) 
{
    // 使能GPIOB时钟
    RCC_APB2ENR_REG |= RCC_APB2ENR_IOPBEN;

    // 配置GPIOB引脚15的模式和配置
    uint32_t temp = GPIOB_CRL_REG;
    temp &= ~(0x0F000000); 				 	// 清除
    temp |= (GPIO_MODE_OUT_50MHZ << 28);  	// 设置MODE15为通用推挽输出模式，最大速度50MHz
    temp &= ~(0x00F00000);  				// 清除CNF15位（第20-23位）
    temp |= (GPIO_CNF_PP << 30);  			// 设置CNF15为推挽输出配置
    GPIOB_CRL_REG = temp;
}
/***********************************************************************************************************
* @描述	: main
***********************************************************************************************************/
int main(void) 
{
    // 配置GPIOB引脚
    GPIOB_ConfigPin15AsOutput();
    // 主循环
    while (1) 
	{
        // 切换GPIOB引脚15的状态
        GPIOB_ODR_REG = GPIO_PIN15;  
        delay(1000000);  // 延时
		// 切换GPIOB引脚15的状态
        GPIOB_ODR_REG &= ~GPIO_PIN15;  
        delay(1000000);  // 延时
    }
}