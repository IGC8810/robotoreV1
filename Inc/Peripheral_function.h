#ifndef Peripheral_function_H
#define Peripheral_function_H

#include "main.h"
#include "AQM0802.h"
#include "ICM_20648.h"
#include "INA260.h"
#include "Linetrace.h"
#include "arm_math.h"
#include "Flash_F405.h"
#include "Filter.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart1;
extern uint16_t line_senLLL;
extern uint16_t line_senLL;
extern uint16_t line_senL;
extern uint16_t line_senR;
extern uint16_t line_senRR;
extern uint16_t line_senRRR;
extern int64_t enc_tim1_total;
extern int64_t enc_tim8_total;
//extern int64_t enc_tim1_cnt;
//extern int64_t enc_tim8_cnt;
extern int64_t enc_cnt;
extern unsigned char main_pattern;
extern float velR;
extern float velL;
extern int posR;
extern int posL;
extern char error_flag;
extern int timer;
extern uint16_t error_cnt;
extern float target_vel;
extern uint8_t maker_check;
extern char crossline_flag;
extern unsigned char velocity_pattern;
extern int encoder_event;
extern uint8_t flash_flag;
extern uint32_t log_adress;
extern uint32_t* flash_read_test;
extern int32_t enc_tim1_cnt_10ms;
extern int32_t enc_tim8_cnt_10ms;
extern uint16_t maker_cnt;
extern uint8_t calibration_flag;
extern int16_t log_array;
extern float PlanVelo[];
extern uint32_t plan_velo_adress;
extern uint8_t second_trace_flag;
extern int64_t enc_tim_total;
extern char setup_mode;
extern float mm_total;
extern int64_t enc_cnt2;
extern uint32_t maker_adress;
extern float maker_distance_L[];
extern float maker_distance_R[];
extern uint16_t maker_distance_cmp_lim;
extern float log_zg;
extern float log_mm;
extern float PlanVelo2[];
extern uint8_t second_trace_pattern;

#define ADC_DATA_BUFFR_SIZE		((uint16_t)16)
#define LED_R_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_G_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_B_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define LED_R_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_G_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_B_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
#define SW_LEFT		 	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)
#define SW_RIGHT	 	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)
#define SW_UP			HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)
#define SW_DOWN		 	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)
#define SW_PUSH		 	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)
#define SW_TACTILE 		HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)
#define MR_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define MR_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define ML_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define ML_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define ENC_PULSE_MM 	0.012207f		//0.024414f 2逓倍
// 速度 = 1msでカウントしたパルス　* 1パルスで進む距離 * 1000 [mm/s]
// 1msで数mm進むのでm/s これに1000かけてmm/s
// 1pulseで進む距離　= タイヤ周長 / (エンコーダのパルス * n逓倍 * 減速比)
//								= 68mm / (512 * 4 * 2.72) = 0.0122...

#define ESC_MAX 		3527	//84[us]
#define ESC_MIN			1763	//42[us]

#define MAX_VELOCITY	6000.0f		//[mm/s]5000
#define MIN_VELOCITY	1100.0f		//[mm/s]1000
#define START_VELOCITY	1100.0f		//[mm/s]1000
#define ACCELERATION	10.0f		//[mm/s^2]50
#define DECELERATION	10.0f		//[mm/s^2]10
#define END_VELOCITY	1000.0f		//[mm/s]1000
#define END_DISTANCE    250.0f		//[mm]

#define ERRORCHECK		25000		//25000←9号館
#define CROSSCHECK		2500		//2500←9号館
#define MAKERTHRESHOLD	1600		//900←9号館
#define LINE_OFFSET_L	0.0f

#define MAX_VELOCITY2	7000.0f		//[mm/s]7000
#define MIN_VELOCITY2	1300.0f		//[mm/s]1200
#define START_VELOCITY2	1300.0f		//[mm/s]1200
#define ACCELERATION2	20.0f		//[mm/s^2]100
#define DECELERATION2	20.0f		//[mm/s^2]10
#define END_VELOCITY2	1400.0f		//[mm/s]1200
#define END_DISTANCE2   250.0f		//[mm]

/*
velo_spline = 0.000000958783462728946f * powf(curvature, 3) + (-0.00361845365313069f) * powf(curvature, 2) + 5.28713093017124f * curvature + 235.903467671659f;
#define MAX_VELOCITY	5000.0f		//[mm/s]
#define MIN_VELOCITY	1100.0f		//[mm/s]1500
#define START_VELOCITY	1200.0f		//[mm/s]1000
#define ACCELERATION	400.0f		//[mm/s^2]500
#define DECELERATION	80.0f		//[mm/s^2]
#define END_VELOCITY	1500.0f		//[mm/s]1000
#define END_DISTANCE    100.0f		//[mm]
*/

void All_init(void);
void ESC_Calibration(void);
void IO_init(void);
void ADval_get(void);
void ADval_sum(void);
void CrossCheck(uint16_t);
void MakerCheck(uint8_t);
uint8_t MakerSenTh(uint16_t);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);
void led_pattern(char);
void IOstate_get(void);
void getEncoder(void);
float mileage(float);
void MotorCtrl(short,short);
void setup(void);
void Calculation_offset_zg(void);
float Velo_Spline_Curve(float);
uint8_t StartGoalCheck(uint8_t);

#endif

/* 2020/02/15 湘南大会
#define MAX_VELOCITY	6000.0f		//[mm/s]
#define MIN_VELOCITY	1100.0f		//[mm/s]
#define START_VELOCITY	1100.0f		//[mm/s]
#define ACCELERATION	10.0f		//[mm/s^2]
#define DECELERATION	10.0f		//[mm/s^2]
#define END_VELOCITY	1000.0f		//[mm/s]
#define END_DISTANCE    250.0f		//[mm]

#define ERRORCHECK		25000		//
#define CROSSCHECK		2500		//
#define MAKERTHRESHOLD	1500		//
#define LINE_OFFSET_L	0.0f

#define MAX_VELOCITY2	7000.0f		//[mm/s]
#define MIN_VELOCITY2	1300.0f		//[mm/s]
#define START_VELOCITY2	1300.0f		//[mm/s]
#define ACCELERATION2	50.0f		//[mm/s^2]
#define DECELERATION2	20.0f		//[mm/s^2]
#define END_VELOCITY2	1400.0f		//[mm/s]
#define END_DISTANCE2   250.0f		//[mm]
*/
