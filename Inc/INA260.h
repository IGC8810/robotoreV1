#ifndef INA260_H
#define INA260_H

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

unsigned short INA260_read(uint8_t);
void INA260_write(uint8_t,uint8_t,uint8_t);
void INA260_init(void);

#endif
