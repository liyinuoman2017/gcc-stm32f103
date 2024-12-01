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
	
