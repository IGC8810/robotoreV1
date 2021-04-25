//AQM0802.h Ver.1.0
#ifndef AQM0802_H
#define AQM0802_H

#include "main.h"
#include <stdio.h>
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c1;

void lcd_cmd(uint8_t);
void lcd_data(uint8_t);
void lcd_init(void);
void lcd_clear(void);
void lcd_locate(int,int);
void lcd_print(const char *);
short lcd_printf(const char *, ...);

#endif
