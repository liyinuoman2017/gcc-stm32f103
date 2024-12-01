# gcc-stm32f103
How to compile stm32f103 programs manually using GCC



![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/0e96a2bff9f64c10874669bdda4728eb.png)

<h1><font color=red>如何不使用任何IDE（集成开发环境）编译stm32程序?</font></h1>

**集成开发环境将编辑器、编译器、链接器、调试器等开发工具集成在一个统一的软件中**，使得开发人员可以更加简单、高效地完成软件开发过程。如果我们不使用KEIL,IAR等集成开发环境，就需要手动完成对stm32程序的编译，得到一个stm32可以执行的文件。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/8ee924d5dd6845d29bb7d933a75baa45.png)


**这个问题看似很闲的蛋疼**，其实不然。集成开发环境**降低了开发难度，隐藏了开发细节**。如果我们能手动实现程序的编译，这将使我们更深入的理解**程序编译**和**stm32启动**。
开发步骤分为以下两个步骤：

> 1、编译环境的搭建 
> 2、编写和编译程序

开发环境：

> Windows10
> stm32f103rb

## 1.搭建编译环境
搭建编译环境的目的是，实现编译stm32的程序代码，得到一个stm32可执行文件。搭建编译环境分为以下两个步骤：

> 1、安装编译器 
> 2、安装make工具

#### 1.1安装编译器
在Windows10系统下适用于STM32的编译器为**gcc-arm-none-eabi** 。
gcc-arm-none-eabi是GNU项目下的软件，是一个**面向ARM架构的芯片的交叉编译器**。交叉编译器是一种特殊的编译器，它能够使得开发者可以在自己的主机（如PC）上编写和编译代码，然后将编译后的二进制代码部署到目标嵌入式系统（如ARM架构的微控制器）上运行。


**前往ARM的官方网站下载gcc-arm-none-eabi**

```c
https://developer.arm.com/downloads/-/gnu-rm
```


![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/405afd99b34847fc80d0158fd94fd8ab.png)



**安装gcc-arm-none-eabi**
使用默认选项安装，在最后的完成界面，一定要勾选“**add path to environment variable**”

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/a93816663a2a44b48984a46e9565ab19.png)

**测试gcc-arm-none-eabi**
按下win+r按键输入cmd启动终端，在终端中输入如下指令，查看gcc版本。安装失败会提示指令为无效指令。安装失败有可能是没有勾选“**add path to environment variable**”

```c
arm-none-eabi-gcc --version
```


![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ed695f2fad7647338716942dda9c6bc1.png)

**成就：完成编译器安装后，我们可以将C程序编译成stm32f103rb能识别的二进制代码。**

#### 1.2安装make工具
安装gcc-arm-none-eabi后我们就可以编译C文件得到stm32可执行文件，那我们是不是开始编译我们的工程了呢？显然不是，此时我们会遇到如下问题：

> **1**、当你的程序只有一个源文件的时候，直接使用gcc命令编译就行。软件工程包含的源文件越来越多采用gcc命令逐个手动去编译，很容易混乱而且工作量大，会让人抓狂。而且各个文件可能还得依赖不同的库，这样命令会变得很长，显然这是不可行的办法。
> 
>  **2**、开发一个项目的时候，进行了一个简单的修改，比如就改了一个if条件，修改后都要重新编译一次，假设整个源码的工程里面的源文件的数量几百个或者上千个，完成所有文件的编译是需要大量时间的，编译半天都有可能，就修改了一个小bug而已，花费这么久的时间，明显工作效率会很低。



![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/3914a3c0dff24265849a7bade52888f6.png)




为了解决上述问题make工具诞生了。**make工具可以看成是一个智能的批处理工具，它本身并没有编译和链接的功能，通过调用makefile文件中用户指定的命令来进行编译和链接的**。
makefile就是一个脚本文件，简单的说就像一首歌的乐谱，make工具就像指挥家，指挥家根据乐谱指挥整个乐团怎么样演奏，make工具就根据 makefile中的命令进行编译和链接的。


**下载make工具**
在这里我不直接安装make工具，我们采样迂回的方式安装make工具。下载**wingm**

```c
https://www.mingw-w64.org/downloads/#winlibscom
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/5cff784591ef435cabfed38c66234409.png)

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/e06f689b21894a738ec0ef2df340d017.png)

**安装wingm**
解压wingm，假设我们的路径是E:\tools\mingw64\bin （**每个人的路径可能不一样**）

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/f514c8f0617f4ae7899e572b31339495.png)

将E:\tools\mingw64\bin路径（**每个人的路径可能不一样**）添加到系统环境变量PATH中，这样我们才可以在任意地方执行mingw64\bin中的指令，按照如下步骤将E:\tools\mingw64\bin添加到PATH中。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/934a741e467b46948bf0bfbadbfdec66.png)

**关键步骤到了：**
**将E:\tools\mingw64\bin\ming32-make.exe 重命名为 c:\MinGw\bin\make.exe**

**测试make工具**
按下win+r按键输入cmd**重新启动终端**（**一定要重新启动终端，终端不会实时更新环境变量**），在终端中输入如下指令，查看make版本。安装失败会提示指令为无效指令。

```c
make -v
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/0ad1392c727c44979f7e2a272fa29842.png)


![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/21c020ca795446098c10ca185e30c159.png)

**至此我们的编译环境搭建完成**，接下来我们就开始编译代码之旅！
## 2.编写和编译程序
#### 2.1使用gcc-arm-none-eabi编译一个C文件
写一个main函数，保存为main.c文件

```c
int main(void) 
{

    while (1) 
	{
        // 主循环代码，例如LED闪烁、串口通信等
        // ...
    }
}
```
在main.c文件目录下运行cmd ，执行如下指令，我们编译得到一个main.o文件。

```c
arm-none-eabi-gcc -c main.c -o main.o
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/33e0371934a7466fa213862ccc29c32c.png)

至此我们可以编译得到一个.o文件，这就是我们的第一步。

#### 2.2写一个Makeflie编译C文件
正如前文说的当只有一个C文件时，可以手动使用gcc指令编译，如果有很多C文件时，手动使用gcc指令去完成编译已经变得不可能，此时就需要用到我们的make工具。接下来我们用make工具实现批量编译。
**我们建立一个src文件夹，和一个Makefile文件，并在src文件夹中创建main.c和test.c两个C文件。**

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/3391f9ef55084f35be627f4473247980.png)


**mian.c内容如下：**

```c
void _exit(int status) __attribute__((weak));
void _exit(int status) 
{
    while (1);
}

int main(void) 
{
    while (1) {
        // 主循环代码，例如LED闪烁
        // ...
    }
}
```
**test.c内容如下：**

```c
void delay(void) 
{
	int i = 0 ;
	for(i = 0 ;i < 1000 ; i++)
	{
		
		
	}
}
```
**Makefile内容如下：**

```c
# 定义编译器
CC = arm-none-eabi-gcc

# 目标
TARGET = my_test

# 列出所有源文件
SRCS = $(wildcard src/*.c) 
 
# 将源文件转换为目标文件列表
OBJS = $(SRCS:.c=.o)
  
# 默认目标
all: $(TARGET)

# 生成文件
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET)
```
然后我们在**Makefile文件所在的路径下**运行cmd，并在终端中输入指令**make**，运行结果如下：


![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/68d35e5b4e4b41f6b1eb6a75584bfb33.png)

**于是成功我们利用make工具实现批量编译C文件。**


#### 2.3编写一个启动文件和链接文件
目前我们已经可以利用make工具实现对文件的批量编译，那么是不是可以将编译得到的文件运行到stm32f103rb芯片上呢？
答案是否定的，要让stm32f103rb芯片运行我们编译得到的执行文件，还有以下3个步骤：

> 1、配置arm-none-eabi-gcc参考，让编译器输出stm32f103rb可执行格式的文件
> 2、编写一个启动文件，让代码能被正常引导
> 3、编写一个链接文件，让告诉编译器将生成的代码放置的合适位置


**配置arm-none-eabi-gcc**
当我们在gcc指令后增加一个参数-mcpu=cortex-m3，此时编译器输出的文件格式将是cortex-m3芯片能执行的格式，而stm32f103rb属于cortex-m3系列。
```c
arm-none-eabi-gcc -c -mcpu=cortex-m3
```
**编写一个启动文件**

stm32芯片启动流程如下：

> 1、用代码段的第1个32位数初始化堆栈指针SP
> 2、用代码段的第2个32位数初始化程序指针PC（也就是程序跳转到第2个32位数的数值指向的地址）


为了完成正确的启动，我们需要写一个汇编格式的启动文件，**汇编代码内容如下**：

```c
/**
*********************************************************************************************************
*                                      (c) Copyright 2024-2032
*                                         All Rights Reserved
* @By      : liwei
*********************************************************************************************************
**/
.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

/******************************************************************************
复位启动函数Reset_Handler 执行跳转到main函数
******************************************************************************/ 
    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:	

	bl	main
	bx	lr    
.size	Reset_Handler, .-Reset_Handler
/******************************************************************************
向量表 第一个值为SP 第二个值为复位地址
******************************************************************************/ 
.global	g_pfnVectors  
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors
    
    
g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	
```
由启动函数可知，我们定义了一个g_pfnVectors向量表，向量表中的 第一个值为SP初始化值， 第二个值为复位Reset_Handler函数地址。
在复位函数Reset_Handler中跳转到了mian函数。


**编写一个链接文件**
**是不是增加一个启动文件，就能让stm32f103rb芯片运行我们编译得到的执行文件了呢？**
**答案是否定的！**

> 问题：**代码应该下载到stm32f103rb芯片的什么位置？**


在编译过程中有一个重要的环节：**链接**

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/34bea5f045cc404c9792abbddc9a9590.png)

链接可以将多个目标代码整合成最终的可执行程序，在链接过程中有个重要的步骤就是**给程序指定一个起始位置**。因此我们编写一个简单的链接文件：

```c
/* 定义flash 和 ram 区域  */

MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(Reset_Handler)


_heap_size = 0;    /* required amount of heap  */
_stack_size = 0;   /* required amount of stack */

/* The stack starts at the end of RAM and grows downwards. Full-descending*/
_estack = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
    /* Reset and ISR vectors */
    .isr_vector :
    {
        __isr_vector_start__ = .;
        KEEP(*(.isr_vector)) /* without 'KEEP' the garbage collector discards this section */
        ASSERT(. != __isr_vector_start__, "The .isr_vector section is empty");
    } >flash


    /* Text section (code and read-only data) */
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        *(.text*)   /* code */
        *(.rodata*) /* read only data */

        /*
         * NOTE: .glue_7 and .glue_7t sections are not needed because Cortex-M
         * only supports Thumb instructions, no ARM/Thumb interworking.
         */

        /* Static constructors and destructors */
        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >flash
    /*
     * Initialized data section. This section is programmed into FLASH (LMA
     * address) and copied to RAM (VMA address) in startup code.
     */
    _sidata = .;
    .data : AT(_sidata) /* LMA address is _sidata (in FLASH) */
    {
        . = ALIGN(4);
        _sdata = .; /* data section VMA address */
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >ram


    /* Uninitialized data section (zeroed out by startup code) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >ram


    /*
     * Reserve memory for heap and stack. The linker will issue an error if
     * there is not enough memory.
     */
    ._heap :
    {
        . = ALIGN(4);
        . = . + _heap_size;
        . = ALIGN(4);
    } >ram
    ._stack :
    {
        . = ALIGN(4);
        . = . + _stack_size;
        . = ALIGN(4);
    } >ram
}

/* Nice to have */
__isr_vector_size__ = SIZEOF(.isr_vector);
__text_size__ = SIZEOF(.text);
__data_size__ = SIZEOF(.data);
__bss_size__ = SIZEOF(.bss);

```
从代码中可以发现，程序定义了flash和ram的起始位置和大小。

```c
flash (rx) : ORIGIN = 0x08000000, LENGTH = 64K
ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
```
程序同时定了一个中断向量段，这个段在flash的开始位置。

```c
    /* Reset and ISR vectors */
    .isr_vector :
    {
        __isr_vector_start__ = .;
        KEEP(*(.isr_vector)) /* without 'KEEP' the garbage collector discards this section */
        ASSERT(. != __isr_vector_start__, "The .isr_vector section is empty");
    } >flash
```
程序还定义了.text 、.data、.bss 、._heap 、._stack段。

**编写Makefile文件**
编写一个makefil文件：

```bash
# toolchain
CC           = arm-none-eabi-gcc
CP           = arm-none-eabi-objcopy
AS           = arm-none-eabi-gcc -x assembler-with-cpp

# all the files will be generated with this name 
PROJECT_NAME=stm32f10x_project

# user specific
SRC      += ./user/main.c
# startup
ASM_SRC  += ./user/startup_stm32f10x_md.s

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)
# Define optimisation level here
MC_FLAGS = -mcpu=cortex-m3
AS_FLAGS = $(MC_FLAGS) -g  -mthumb  
CP_FLAGS = $(MC_FLAGS)  -g  -mthumb  -Wall -fverbose-asm 
LD_FLAGS = $(MC_FLAGS) -g  -mthumb  -Xlinker --gc-sections -T stm32_flash.ld 

# makefile rules
all: $(OBJECTS) $(PROJECT_NAME).elf  $(PROJECT_NAME).hex $(PROJECT_NAME).bin
	 arm-none-eabi-size $(PROJECT_NAME).elf

%.o: %.c
	$(CC) -c $(CP_FLAGS) -I . $(INC_DIR) $< -o $@

%.o: %.s
	$(AS) -c $(AS_FLAGS) $< -o $@

%.elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@

%.hex: %.elf
	$(CP) -O ihex $< $@

%.bin: %.elf
	$(CP) -O binary -S  $< $@

clean:
	del /Q  $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin


```
**编写main文件**
程序需要在main函数中实现对stm32f103rb芯片的控制，为了直观的显示结果，程序通过一个gpio控制一个led灯，main函数如下：

```bash
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
```
程序通过直接控制寄存器的方式控制GPIOB引脚15的状态，从而实现对led状态的控制。
在Makefile文件路径下运行cmd,在终端中执行make指令，我们将得到.hex的可执行文件，将执行文件下载到stm32f103rb中可以发现被控制的led灯闪烁。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/9432b34afbdf47d7828f3fcdf54c4f61.png)

**至此，我们已经基本完成目标，使用gcc工具编译得到了一个stm32f103rb可执行文件，大功告成！**

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/21c020ca795446098c10ca185e30c159.png)

#### 2.4完善启动文件
虽然我们实现了目标，但是实际上目前完成的工程只是一个简单的，不完善的工程，接下来我们继续完善启动文件，深入理解芯片启动的完整过程。
启动代码如下：

```bash
/**
*********************************************************************************************************
*                                      (c) Copyright 2024-2032
*                                         All Rights Reserved
* @By      : liwei
*********************************************************************************************************
**/
    
  .syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the . defined in linker script */
.word	_sidata
.word	_sdata
.word	_edata
.word	_sbss
.word	_ebss

.equ  BootRAM, 0xF108F85F

/******************************************************************************
复位启动函数Reset_Handler 
******************************************************************************/ 
    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:	

/* 将初始化数据段器从FLASH复制到SRAM*/  
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4
    
LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* 初始化 bss */  
FillZerobss:
	movs	r3, #0
	str	r3, [r2], #4
    
LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss
/* 跳转到main函数*/
	bl	main
	bx	lr    
.size	Reset_Handler, .-Reset_Handler

/******************************************************************************
默认中断代替函数
******************************************************************************/ 
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
向量表 
******************************************************************************/    
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors
    
    
g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
	.word	WWDG_IRQHandler
	.word	PVD_IRQHandler
	.word	TAMPER_IRQHandler
	.word	RTC_IRQHandler
	.word	FLASH_IRQHandler
	.word	RCC_IRQHandler
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	DMA1_Channel1_IRQHandler
	.word	DMA1_Channel2_IRQHandler
	.word	DMA1_Channel3_IRQHandler
	.word	DMA1_Channel4_IRQHandler
	.word	DMA1_Channel5_IRQHandler
	.word	DMA1_Channel6_IRQHandler
	.word	DMA1_Channel7_IRQHandler
	.word	ADC1_2_IRQHandler
	.word	USB_HP_CAN1_TX_IRQHandler
	.word	USB_LP_CAN1_RX0_IRQHandler
	.word	CAN1_RX1_IRQHandler
	.word	CAN1_SCE_IRQHandler
	.word	EXTI9_5_IRQHandler
	.word	TIM1_BRK_IRQHandler
	.word	TIM1_UP_IRQHandler
	.word	TIM1_TRG_COM_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	TIM3_IRQHandler
	.word	TIM4_IRQHandler
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	I2C2_EV_IRQHandler
	.word	I2C2_ER_IRQHandler
	.word	SPI1_IRQHandler
	.word	SPI2_IRQHandler
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	USART3_IRQHandler
	.word	EXTI15_10_IRQHandler
	.word	RTCAlarm_IRQHandler
	.word	USBWakeUp_IRQHandler	
  .word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	BootRAM         
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
*
*******************************************************************************/
    
  .weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler
	
  .weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler
	
  .weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler
	
  .weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler

	.weak	TAMPER_IRQHandler
	.thumb_set TAMPER_IRQHandler,Default_Handler

	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak	ADC1_2_IRQHandler
	.thumb_set ADC1_2_IRQHandler,Default_Handler

	.weak	USB_HP_CAN1_TX_IRQHandler
	.thumb_set USB_HP_CAN1_TX_IRQHandler,Default_Handler

	.weak	USB_LP_CAN1_RX0_IRQHandler
	.thumb_set USB_LP_CAN1_RX0_IRQHandler,Default_Handler

	.weak	CAN1_RX1_IRQHandler
	.thumb_set CAN1_RX1_IRQHandler,Default_Handler

	.weak	CAN1_SCE_IRQHandler
	.thumb_set CAN1_SCE_IRQHandler,Default_Handler

	.weak	EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler

	.weak	TIM1_BRK_IRQHandler
	.thumb_set TIM1_BRK_IRQHandler,Default_Handler

	.weak	TIM1_UP_IRQHandler
	.thumb_set TIM1_UP_IRQHandler,Default_Handler

	.weak	TIM1_TRG_COM_IRQHandler
	.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak	TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler

	.weak	EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler

	.weak	RTCAlarm_IRQHandler
	.thumb_set RTCAlarm_IRQHandler,Default_Handler

	.weak	USBWakeUp_IRQHandler
	.thumb_set USBWakeUp_IRQHandler,Default_Handler

```

首先程序定义了一个完整了向量表，它不仅包含SP初始化值、PC初始化值、同时还包括stm32f103rb芯片所有的中断向量。

> g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.
	.
	.

在复位函数Reset_Handler中，完成了对data 段和 bss段的初始化，最后跳转到mian函数。

```bash
/******************************************************************************
复位启动函数Reset_Handler 
******************************************************************************/ 
    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:	

/* 将初始化数据段器从FLASH复制到SRAM*/  
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4
    
LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* 初始化 bss */  
FillZerobss:
	movs	r3, #0
	str	r3, [r2], #4
    
LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss
/* 跳转到main函数*/
	bl	main
	bx	lr    
.size	Reset_Handler, .-Reset_Handler
```



> 问题：**程序中的初始化静态变量的初始值是怎么来的？**
> 如全局变量：int value = 1314;
> value是保存在ram中的变量，芯片重启后ram中的值是随机的，程序是怎么让ram中的值为1314的？


![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/19e000d940ac401f9c01da3668f7295b.png)

利用分散加载技术，程序将存放在ROM中的data 段数据复制到RAM指定的位置，从而实现了data 段的数据初始化。Reset_Handler中的如下代码完成了数据加载功能：

```bash
/* 将初始化数据段器从FLASH复制到SRAM*/  
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4
    
LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
```
在Makefile文件路径下运行cmd,在终端中执行make指令，我们将得到.hex的可执行文件，将执行文件下载到stm32f103rb中可以发现被控制的led灯闪烁。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/6d5fb2c2fc9a464db9f8b214a978de48.png)

#### 2.5完善工程
我们还剩下最后一个问题：

> 使用st官方的库进行编程（不使直接用寄存器地址编程）
此时我们要添加st的官方库文件，修改Makefile文件，修改mian.c文件。

**mian.c内容如下：**
```bash
/**
*********************************************************************************************************
*                                      (c) Copyright 2024-2032
*                                         All Rights Reserved
* @By      : liwei
*********************************************************************************************************
**/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/***********************************************************************************************************
* @描述	: 延时函数
***********************************************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
/***********************************************************************************************************
* @描述	: 设置使能
***********************************************************************************************************/
void RCC_Configuration(void)
{
    /* GPIOA, GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
}
/***********************************************************************************************************
* @描述	: 配置GPIO
***********************************************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/***********************************************************************************************************
* @描述	: main函数
***********************************************************************************************************/
int main(void)
{

    RCC_Configuration();
    GPIO_Configuration();
    //LED闪烁
    while (1)
	{
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        Delay(300000);
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        Delay(300000);			
	}
}
```
**Makefile内容如下：**

```bash
# toolchain
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp
# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJECT_NAME=stm32f10x_project

# define include dir
INCLUDE_DIRS =

# define stm32f10x lib dir
STM32F10x_LIB_DIR      = ./stm32f10x_lib

# define user dir
USER_DIR     = ./user

# link file
LINK_SCRIPT  = ./stm32_flash.ld

# user specific
SRC       =
ASM_SRC   =
SRC      += $(USER_DIR)/main.c
# user include
INCLUDE_DIRS  = $(USER_DIR)

# source director
STM32F1_STD_LIB     = $(STM32F10x_LIB_DIR)/STM32F10x_StdPeriph_Driver
STM32F1_CORE_DIR    = $(STM32F10x_LIB_DIR)/CMSIS/CM3/CoreSupport
STM32F1_DEVICE_DIR  = $(STM32F10x_LIB_DIR)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
STM32F1_SRC_DIR     = $(STM32F1_STD_LIB)/src
STM32F1_INC_DIR     = $(STM32F1_STD_LIB)/inc

# startup
ASM_SRC  += $(STM32F1_DEVICE_DIR)/startup/gcc_ride7/startup_stm32f10x_md.s

# CMSIS
SRC  += $(STM32F1_DEVICE_DIR)/system_stm32f10x.c
SRC  += $(STM32F1_CORE_DIR)/core_cm3.c

# use libraries, please add or remove when you use or remove it.
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_rcc.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_gpio.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_exti.c
SRC  += $(STM32F1_SRC_DIR)/stm32f10x_usart.c
SRC  += $(STM32F1_SRC_DIR)/misc.c

# include directories
INCLUDE_DIRS += $(STM32F1_CORE_DIR)
INCLUDE_DIRS += $(STM32F1_DEVICE_DIR)
INCLUDE_DIRS += $(STM32F1_INC_DIR)
INCLUDE_DIRS += $(STM32F1_STD_LIB)
INC_DIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))
OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)

# Define optimisation level here
MC_FLAGS = -mcpu=cortex-m3
AS_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.s=.lst)
CP_FLAGS = $(MC_FLAGS) -Os -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) 
LD_FLAGS = $(MC_FLAGS) -g -gdwarf-2 -mthumb -nostartfiles -Xlinker --gc-sections -T$(LINK_SCRIPT) -Wl,-Map=$(PROJECT_NAME).map,--cref,--no-warn-mismatch

# makefile rules
all: $(OBJECTS) $(PROJECT_NAME).elf  $(PROJECT_NAME).hex $(PROJECT_NAME).bin
	$(TOOLCHAIN)size $(PROJECT_NAME).elf

%.o: %.c
	$(CC) -c $(CP_FLAGS) -I . $(INC_DIR) $< -o $@

%.o: %.s
	$(AS) -c $(AS_FLAGS) $< -o $@

%.elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@

%.hex: %.elf
	$(CP) -O ihex $< $@

%.bin: %.elf
	$(CP) -O binary -S  $< $@

clean:
	del /Q  $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

```
在Makefile文件路径下运行cmd,在终端中执行make指令，我们将得到.hex的可执行文件，将执行文件下载到stm32f103rb中可以发现被控制的led灯闪烁。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/96391608277245debf00ead65edf71c9.png)


## 3.总结
为了实现如何不使用任何IDE集成软件编译得到stm32程序的执行文件，安装了GCC和make工具。
为了编译得到stm32f103rb执行文件，需要完成以下操作：

> 1、编写启动文件
>  2、编写链接文件 
>  3、编写Makeflie文件 
>  4、编写C语言源代码

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c5d2c2e365634ab0916c41ff953234b7.png)


**希望获取源码的朋友可以在评论区留言**
**希望获取源码的朋友可以在评论区留言**
**希望获取源码的朋友可以在评论区留言**


**创作不易希望朋友们点赞，转发，评论，关注!
您的点赞，转发，评论，关注将是我持续更新的动力!
CSDN：https://blog.csdn.net/li_man_man_man
今日头条：https://www.toutiao.com/article/7149576260891443724**
