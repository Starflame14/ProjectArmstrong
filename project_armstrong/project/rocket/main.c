/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "drivers/vga.h"

int main() {
	vga_fill_area(0, 0, 511, 127, vga_color(0, 0, 0));
	vga_terminal_printf("It's not rocket science\n");

	while (1) {
	}
}
