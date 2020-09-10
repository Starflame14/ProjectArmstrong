/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "interrupt.h"
#include <stdint.h>

#define NVIC_ISER	((volatile uint32_t*) 0xE000E100)

void interrupt_enable(interrupt_number_t irq_number) {
	NVIC_ISER[irq_number >> 5] |= 1 << (irq_number & 0x1f);
}

void interrupt_disable(interrupt_number_t irq_number) {
	NVIC_ISER[irq_number >> 5] &= ~(1 << (irq_number & 0x1f));
}
