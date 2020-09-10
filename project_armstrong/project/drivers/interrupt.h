/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef COMMON_FVP_INTERRUPT_H_
#define COMMON_FVP_INTERRUPT_H_

/**
 * Platform specific interrupt request numbers
 */
typedef enum interrupt_number {
	IRQnum_UART0_RX = 0,
	IRQnum_UART0_TX,
	IRQnum_UART1_RX,
	IRQnum_UART1_TX,
	IRQnum_UART2_RX,
	IRQnum_UART2_TX,
	IRQnum_GPIO0_2,
	IRQnum_GPIO1_3,
	IRQnum_Timer0,
	IRQnum_Timer1,
	IRQnum_DualTimer,
	IRQnum_SPI1_num,
	IRQnum_UART012_overflow,
	IRQnum_Ethernet,
	IRQnum_AudioI2S,
	IRQnum_TouchScreen,
} interrupt_number_t;

/**
 * @brief      Enables interrupt channel
 *
 * @param[in]  irq_number  The interrupt request number
 */
void interrupt_enable(interrupt_number_t irq_number);

/**
 * @brief      Disables interrupt channel
 *
 * @param[in]  irq_number  The interrupt request number
 */
void interrupt_disable(interrupt_number_t irq_number);

#endif /* COMMON_FVP_INTERRUPT_H_ */
