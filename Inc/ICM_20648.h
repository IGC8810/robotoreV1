//ICM_20648.h Ver.1.2
#ifndef ICM_20648_H
#define ICM_20648_H

#include "main.h"

extern SPI_HandleTypeDef hspi1;
// IMUから取得したデータ
extern volatile int16_t 	xa, ya, za; // 加速度(16bitデータ)
extern volatile int16_t 	xg, yg, zg;	// 角加速度(16bitデータ)

#define CS_RESET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define CS_SET   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)

#define ACCEL_RANGE		2.0f		// 2[g]
#define GYRO_RANGE		2000.0f		// 2000[deg/s]
#define MAXDATA_RANGE	32764.0f	// 16bitデータの最大値
#define G_ACCELERATION	9.80665.0f	// G

uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
uint8_t IMU_init(void);
void read_zg_data(void);
void read_gyro_data(void);
void read_xa_data(void);
void read_accel_data(void);

/*

To convert the output of the gyroscope to angular rate measurement use the formula below:

Z_angular_rate = GYRO_ZOUT/Gyro_Sensitivity
			   = zg/ScaleFactor[deg/s];

Gyro		Range(dps)	ScaleFactor(LSB/dps)
					±250				131
					±500				65.5
					±1000				32.8
					±2000				16.4

To convert the output of the accelerometer to acceleration measurement use the formula below:

Z_acceleration = ACCEL_ZOUT/Accel_Sensitivity
			   = za/ScaleFactor[G];

Accel		Range(g)	ScaleFactor(LSB/g)
					±2					16384
					±4					8192
					±8					4096
					±16					2048
*/

#endif
