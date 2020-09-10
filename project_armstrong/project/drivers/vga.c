/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vga.h"
#include <stdarg.h>
#include <stdio.h>

#define VGA_CONSOLE		(*(volatile uint8_t*)0x41000000)
#define VGA_IMAGE_PIXEL	((volatile uint32_t*)0x41100000)

void vga_set_pixel(uint16_t x, uint16_t y, uint16_t color) {
	if (x >= VGA_WIDTH || y >= VGA_HEIGHT) {
		return;
	}

	VGA_IMAGE_PIXEL[VGA_WIDTH * (VGA_HEIGHT - y - 1) + x] = color;
}

void vga_fill_area(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	uint16_t x;
	uint32_t offset;

	if (x0 >= VGA_WIDTH || x1 >= VGA_WIDTH || y0 >= VGA_HEIGHT || y1 >= VGA_HEIGHT) {
		return;
	}

	offset = y0 * VGA_WIDTH + x0;
	for (; y0 <= y1; y0++) {
		for (x = x0; x <= x1; x++) {
			VGA_IMAGE_PIXEL[offset + x] = color;
		}
		offset += VGA_WIDTH;
	}
}

void vga_terminal_putc(char c) {
	VGA_CONSOLE = c;
}

void vga_terminal_printf(const char *fmt, ...) {
	char buffer[128];
	char *str = buffer;
	va_list args;

	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	while (*str) {
		VGA_CONSOLE = *str++;
	}
}
