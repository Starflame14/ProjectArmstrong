/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

	.syntax unified
	.cpu cortex-m0
	.fpu softvfp
	.thumb

	.global  isr_vector
	.global  Default_Handler

	.word  _sidata
	.word  _sdata
	.word  _edata
	.word  __bss_start__
	.word  __bss_end__


	.section .text.Reset_Handler
	.weak Reset_Handler
	.type Reset_Handler, %function
Reset_Handler:
	ldr r0, =_sidata
	ldr r1, =_sdata
	ldr r2, =_edata
	b 2f
1:
	ldr r3, [r0]
	str r3, [r1]
	adds r0, r0, #4
	adds r1, r1, #4
2:
	cmp r1, r2
	bne 1b

	/* Fill BSS with zeros */
	ldr r0, = __bss_start__
	ldr r1, = __bss_end__
	movs r2, #0
	b 2f
1:
	str r2, [r0]
	adds r0, r0, #4
2:
	cmp r0, r1
	bne 1b

	bl SystemInit
	bl __libc_init_array
	bl main
	wfe

	.size Reset_Handler, .-Reset_Handler
	.section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b  Infinite_Loop
	.size  Default_Handler, .-Default_Handler

	.section .isr_vector,"a",%progbits
	.type isr_vector, %object
	.size isr_vector, .-isr_vector

isr_vector:
	.word _estack
	.word Reset_Handler
	.word NMI_Handler
	.word HardFault_Handler
	.word MemManage_Handler
	.word BusFault_Handler
	.word UsageFault_Handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word SVC_Handler
	.word DebugMon_Handler
	.word 0
	.word PendSV_Handler
	.word SysTick_Handler

	.word UART0_RX_IRQHandler
	.word UART0_TX_IRQHandler
	.word UART1_RX_IRQHandler
	.word UART1_TX_IRQHandler
	.word UART2_RX_IRQHandler
	.word UART2_TX_IRQHandler
	.word GPIO0_2_IRQHandler
	.word GPIO1_3_IRQHandler
	.word Timer0_IRQHandler
	.word Timer1_IRQHandler
	.word DualTimer_IRQHandler
	.word SPI1_Handler
	.word UART012_overflow_IRQHandler
	.word Ethernet_IRQHandler
	.word AudioI2S_IRQHandler
	.word TouchScreen_IRQHandler

	/* Weak aliases for the interrupt handlers */
	.weak NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

	.weak HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak UART0_RX_IRQHandler
	.thumb_set UART0_RX_IRQHandler,Default_Handler

	.weak UART0_TX_IRQHandler
	.thumb_set UART0_TX_IRQHandler,Default_Handler

	.weak UART1_RX_IRQHandler
	.thumb_set UART1_RX_IRQHandler,Default_Handler

	.weak UART1_TX_IRQHandler
	.thumb_set UART1_TX_IRQHandler,Default_Handler

	.weak UART2_RX_IRQHandler
	.thumb_set UART2_RX_IRQHandler,Default_Handler

	.weak UART2_TX_IRQHandler
	.thumb_set UART2_TX_IRQHandler,Default_Handler

	.weak GPIO0_2_IRQHandler
	.thumb_set GPIO0_2_IRQHandler,Default_Handler

	.weak GPIO1_3_IRQHandler
	.thumb_set GPIO1_3_IRQHandler,Default_Handler

	.weak Timer0_IRQHandler
	.thumb_set Timer0_IRQHandler,Default_Handler

	.weak Timer1_IRQHandler
	.thumb_set Timer1_IRQHandler,Default_Handler

	.weak DualTimer_IRQHandler
	.thumb_set DualTimer_IRQHandler,Default_Handler

	.weak SPI1_Handler
	.thumb_set SPI1_Handler,Default_Handler

	.weak UART012_overflow_IRQHandler
	.thumb_set UART012_overflow_IRQHandler,Default_Handler

	.weak Ethernet_IRQHandler
	.thumb_set Ethernet_IRQHandler,Default_Handler

	.weak AudioI2S_IRQHandler
	.thumb_set AudioI2S_IRQHandler,Default_Handler

	.weak TouchScreen_IRQHandler
	.thumb_set TouchScreen_IRQHandler,Default_Handler
