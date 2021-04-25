#ifndef Linetrace_H
#define Linetrace_H

#include "main.h"
#include "Peripheral_function.h"

extern float order_posR;
extern float order_posL;
extern float order_velR;
extern float order_velL;

#define DELTA_T 0.001f

void ErrorCheck(uint16_t);
void posPID(void);
void velPID(float);
//void velPID(float, float);

#endif
