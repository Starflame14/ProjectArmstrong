/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef COMMON_FVP_VGA_H_
#define COMMON_FVP_VGA_H_

#include <stdint.h>

#define VGA_WIDTH 	(512)
#define VGA_HEIGHT	(128)

/**
 * @brief      Builds a 16 bit pixel value from RGB components.
 *
 * @param[in]  r     8 bit red component
 * @param[in]  g     8 bit green component
 * @param[in]  b     8 bit blue component
 *
 * @return     Raw 16 bit value witch is expected by the VGA peripheral.
 */
static inline uint16_t vga_color(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xf0) << 4) | (g & 0xf0) | ((b & 0xf0) >> 4);
}

/**
 * @brief      Sets the pixel of the VGA screen.
 *
 * @param[in]  x      X coordinate (grows from left to right)
 * @param[in]  y      Y coordinate (grows from up to down)
 * @param[in]  color  Raw 16 bit color value
 */
void vga_set_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief      Fills area with the given color
 *
 * @param[in]  x0     Top left X coordinate
 * @param[in]  y0     Top left Y coordinate
 * @param[in]  x1     Bottom right X coordinate
 * @param[in]  y1     Bottom right Y coordinate
 * @param[in]  color  Raw 16 bit color value
 */
void vga_fill_area(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief      Puts a character to the VGA terminal
 *
 * @param[in]  c     Character
 */
void vga_terminal_putc(char c);

/**
 * @brief      Prints a formatted string to the VGA terminal
 *
 * @param[in]  str   The string
 */
void vga_terminal_printf(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

#endif /* COMMON_FVP_VGA_H_ */
