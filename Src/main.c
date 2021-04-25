/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Peripheral_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  All_init();
  int tim_buf;
  int16_t log_cmp;
  float log_buf;
  float velo_temp;
  float mm_buf;
  uint32_t end_log_adress;
  uint8_t buf = 0;
  float spline_temp;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(main_pattern) {
			case 0:	//setup
				setup();
				break;
			case 10:
				//maker_adress = start_adress_sector11;
				//maker_cnt = 0;
				/*if(second_trace_flag == 1) {
					while(isnan(*(float*)maker_adress) == 0){
						maker_distance_L[maker_cnt] = *(float*)maker_adress;
						maker_adress += 0x04;
						maker_distance_R[maker_cnt] = *(float*)maker_adress;
						maker_adress += 0x04;
						maker_cnt++;
					}
					maker_distance_cmp_lim = maker_cnt;
				}*/

				/*target_vel = 600.0f;
				if(mileage((float)enc_cnt) >= 300) {
					main_pattern++;
					timer = 0;
					target_vel = 0.0f;
				}*/
				maker_adress = start_adress_sector9;

				timer = 0;
				target_vel = 0.0f;
				main_pattern++;

				break;
			case 11:	//count down
				if(timer >= 0 && timer < 1000) ;
				else if(timer >= 1000 && timer < 2000) ;
				else if(timer >= 2000 && timer < 3000) {
					calibration_flag = 1;
					led_pattern(2);
				}
				else if(timer >= 3000 && timer < 4000) led_pattern(4);
				else if(timer >= 4000 && timer < 5000) {
					led_pattern(1);
					calibration_flag = 0;
					Calculation_offset_zg();
				}
				else if(timer >= 5000) {
					if (second_trace_flag == 0){
						if (velocity_pattern == 1) target_vel = 1000.0f;
						else if (velocity_pattern == 2) target_vel = 1200.0f;
						else if (velocity_pattern == 3) target_vel = 1500.0f;
						else target_vel = 0.0f;
						led_pattern(0);
						main_pattern++;
					}
					else {
						//target_vel = START_VELOCITY;
						if(second_trace_pattern == 1) target_vel = START_VELOCITY;
						else if(second_trace_pattern == 2) target_vel = START_VELOCITY2;
						else target_vel = 1000.0f;

						main_pattern++;
					}
				}

				break;
			case 12:	//start
				if(maker_check >= 8) {	//start_maker_check
				//if( 1 <= maker_check && maker_check <= 3) {	//start_maker_check
					enc_tim1_total = 0;
					enc_tim8_total = 0;
					enc_tim_total = 0;
					mm_total = 0;
					timer = 0;
					main_pattern = 13;
					enc_tim1_cnt_10ms = 0;
					enc_tim8_cnt_10ms = 0;
					log_adress = start_adress_sector7;
					led_pattern(6);

					if( second_trace_flag == 0 ) flash_flag = 1;
					if( second_trace_flag == 1 ) {
						if(second_trace_pattern == 2)plan_velo_adress = start_adress_sector11;
						else plan_velo_adress = start_adress_sector10;

						target_vel = *(float*)plan_velo_adress;
						plan_velo_adress += 0x04;
						log_adress += 0x04;
						mm_total = *(float*)log_adress;
						log_adress += 0x08;
						/*lcd_locate(0,0);
						lcd_printf("%f",mm_total);
						lcd_locate(0,1);
						lcd_printf("%f",target_vel);*/
					}
				}
				break;
			case 13:	//running
				while( ( mm_total <= mileage((float)enc_tim_total)) && second_trace_flag == 1 ) {
					if(isnan(*(float*)log_adress) != 0) {
						led_pattern(7);
						enc_cnt = 0;
						main_pattern = 14;
						break;
					}
					else mm_total += *(float*)log_adress;

					if(isnan(*(float*)plan_velo_adress) != 0) {
						led_pattern(7);
						enc_cnt = 0;
						main_pattern = 14;
						break;
					}
					else target_vel = *(float*)plan_velo_adress;

					plan_velo_adress += 0x04;
					log_adress += 0x08;
				}
				/*lcd_locate(0,0);
				lcd_printf("%f",mm_total);
				lcd_locate(0,1);
				lcd_printf("%f",target_vel);*/


				/*if(target_vel < 1000 ) led_pattern(7);
				else if(target_vel >= 1000 && target_vel < 1500 ) led_pattern(6);
				else if(target_vel >= 1500 && target_vel < 2000 ) led_pattern(5);
				else if(target_vel >= 2000 && target_vel < 2500 ) led_pattern(4);
				else if(target_vel >= 2500 && target_vel < 3000 ) led_pattern(3);
				else if(target_vel >= 3000 && target_vel < 3500 ) led_pattern(2);
				else if(target_vel >= 3500 ) led_pattern(1);*/

				if(maker_check >= 8 && timer >= 1000) { //goal_maler_check
				//if( 1 <= maker_check && maker_check <= 3 && second_trace_flag == 0 && timer >= 800) {
					/*buf++;
					timer = 0;
					if(buf == 2){
						flash_flag = 0;
						tim_buf = timer;
						led_pattern(4);
						enc_cnt = 0;
						main_pattern = 14;
					}*/
					flash_flag = 0;
					tim_buf = timer;
					led_pattern(4);
					enc_cnt = 0;
					main_pattern++;
					timer = 0;
				}

				break;
			case 14:	//goal
				led_pattern(7);
				//target_vel = END_VELOCITY;
				if(second_trace_pattern == 1) target_vel = END_VELOCITY;
				else if(second_trace_pattern == 2) target_vel = END_VELOCITY2;
				else target_vel = 1000.0f;

				if (mileage((float)enc_cnt) >= 400) {
					target_vel = 0.0f;
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
					led_pattern(7);
					main_pattern = 20;
				}
				break;
			case 20:
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				lcd_locate(0,0);
				lcd_printf("%8d",tim_buf);
				lcd_locate(0,1);
				lcd_printf("%f",mileage((float)(enc_tim1_total + enc_tim8_total) / 2.0f));
				if( second_trace_flag == 0 ) main_pattern++;
				end_log_adress = log_adress;
				break;
			case 21:	// goal„Åó„Åü„ÇâË®àÁîªÈÄüÂ∫¶ÁÆóÔøΩ???øΩ?øΩ
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				led_pattern(1);

				//memcpy(PlanVelo2,PlanVelo,log_array);
				for(log_cmp = 0; log_cmp < log_array ;log_cmp++){
					PlanVelo2[log_cmp] = PlanVelo[log_cmp];
				}

				// for??øΩ?øΩ?ÂÆπ??øΩ?øΩ?Êõ≤??øΩ?øΩ?ÂçäÔøΩ?„Åã„ÇâÈÄüÂ∫¶Ë®àÁîª(‰ªÆ)„Çí‰ΩúÔøΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ?È´òÔøΩ???øΩ?øΩÂ∫¶Âà∂ÈôêÂê´??øΩ?øΩ?
				for(log_cmp = 0; log_cmp < log_array ;log_cmp++) {
					log_buf = PlanVelo[log_cmp];
					spline_temp = Velo_Spline_Curve(log_buf);
					if(spline_temp >= MAX_VELOCITY) PlanVelo[log_cmp] = MAX_VELOCITY;
					else if(spline_temp < MIN_VELOCITY) PlanVelo[log_cmp] = MIN_VELOCITY;
					else PlanVelo[log_cmp] = spline_temp;
				}

				// for??øΩ?øΩ?ÂÆπ??øΩ?øΩ?Êõ≤??øΩ?øΩ?ÂçäÔøΩ?„Åã„ÇâÈÄüÂ∫¶Ë®àÁîª(‰ªÆ)„Çí‰ΩúÔøΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ?È´òÔøΩ???øΩ?øΩÂ∫¶Âà∂ÈôêÂê´??øΩ?øΩ?
				for(log_cmp = 0; log_cmp < log_array ;log_cmp++) {
					log_buf = PlanVelo2[log_cmp];
					spline_temp = Velo_Spline_Curve(log_buf);
					if(spline_temp >= MAX_VELOCITY2) PlanVelo2[log_cmp] = MAX_VELOCITY2;
					else if(spline_temp < MIN_VELOCITY2) PlanVelo2[log_cmp] = MIN_VELOCITY2;
					else PlanVelo2[log_cmp] = spline_temp;
				}

				// for??øΩ?øΩ?ÂÆπ ??øΩ?øΩ?ÈÄüÂèØÂê¶Âà§??øΩ?øΩ? ‰∏çÂèØ„Å™„ÇâÔøΩ???øΩ?øΩÂ∫¶Êõ¥Êñ∞
				log_adress = start_adress_sector7;
				log_adress += 0x04;	//zg„Å®mm„ÅÆ??øΩ?øΩ?„Éº„Çø„Åå‰∫§‰∫í„Å´ÂÖ•„Å£„Å¶??øΩ?øΩ?„ÇãÔøΩ???øΩ?øΩ„Åß„Åö„Çâ??øΩ?øΩ?

				for(log_cmp = 0; log_cmp < log_array ;log_cmp++) {
					mm_buf = *(float*)log_adress;
					if(log_cmp == 0) {	// 1Áï™ÁõÆ„ÅØSTART_VELOCITY„Å®ÊØîÔøΩ?
						if( PlanVelo[log_cmp] > sqrtf( 2.0f * ACCELERATION * mm_buf ) + START_VELOCITY ) PlanVelo[log_cmp] = sqrtf( 2.0f * ACCELERATION * mm_buf ) + START_VELOCITY;
					}
					else {	// 1Âå∫ÈñìÂâç„ÅÆÈÄüÂ∫¶„Å®ÊØîÔøΩ?
						if( PlanVelo[log_cmp] > sqrtf( 2.0f * ACCELERATION * mm_buf ) + PlanVelo[log_cmp - 1] ) PlanVelo[log_cmp] = sqrtf( 2.0f * ACCELERATION * mm_buf ) + PlanVelo[log_cmp - 1];
					}

					log_adress += 0x08;

				}

				// for??øΩ?øΩ?ÂÆπ ??øΩ?øΩ?ÈÄüÂèØÂê¶Âà§??øΩ?øΩ? ‰∏çÂèØ„Å™„ÇâÔøΩ???øΩ?øΩÂ∫¶Êõ¥Êñ∞
				log_adress = start_adress_sector7;
				log_adress += 0x04;	//zg„Å®mm„ÅÆ??øΩ?øΩ?„Éº„Çø„Åå‰∫§‰∫í„Å´ÂÖ•„Å£„Å¶??øΩ?øΩ?„ÇãÔøΩ???øΩ?øΩ„Åß„Åö„Çâ??øΩ?øΩ?

				for(log_cmp = 0; log_cmp < log_array ;log_cmp++) {
					mm_buf = *(float*)log_adress;
					if(log_cmp == 0) {	// 1Áï™ÁõÆ„ÅØSTART_VELOCITY„Å®ÊØîÔøΩ?
						if( PlanVelo2[log_cmp] > sqrtf( 2.0f * ACCELERATION2 * mm_buf ) + START_VELOCITY2 ) PlanVelo2[log_cmp] = sqrtf( 2.0f * ACCELERATION2 * mm_buf ) + START_VELOCITY2;
					}
					else {	// 1Âå∫ÈñìÂâç„ÅÆÈÄüÂ∫¶„Å®ÊØîÔøΩ?
						if( PlanVelo2[log_cmp] > sqrtf( 2.0f * ACCELERATION2 * mm_buf ) + PlanVelo2[log_cmp - 1] ) PlanVelo2[log_cmp] = sqrtf( 2.0f * ACCELERATION2 * mm_buf ) + PlanVelo2[log_cmp - 1];
					}

					log_adress += 0x08;

				}

				log_adress = end_log_adress;
				log_adress -= 0x04;

				// for??øΩ?øΩ?ÂÆπ Ê∏õÔøΩ???øΩ?øΩÂèØÂê¶Âà§??øΩ?øΩ? ‰∏çÂèØ„Å™„ÇâÔøΩ???øΩ?øΩÂ∫¶Êõ¥Êñ∞
				for(log_cmp = log_array - 1; log_cmp >= 0 ;log_cmp--) {

					if(log_cmp != log_array - 1) mm_buf = *(float*)log_adress;

					if(log_cmp == log_array - 1) {	// goalÂú∞ÁÇπ„ÅØEND_VELOCITY„Å®ÊØîÔøΩ?
						if( END_VELOCITY < PlanVelo[log_cmp] - sqrtf( 2.0f * DECELERATION * END_DISTANCE ) ) PlanVelo[log_cmp] = END_VELOCITY + sqrtf( 2.0f * DECELERATION * END_DISTANCE ) ;
					}
					else{	// 1Âå∫ÈñìÔøΩ???øΩ?øΩ??øΩ?øΩ???øΩ?øΩÈÄüÂ∫¶„Å®ÊØîÔøΩ?
						if( PlanVelo[log_cmp + 1] < PlanVelo[log_cmp] - sqrtf( 2.0f * DECELERATION * mm_buf ) ) PlanVelo[log_cmp] = PlanVelo[log_cmp + 1] + sqrtf( 2.0f * DECELERATION * mm_buf );
					}

					log_adress -= 0x08;
				}

				log_adress = end_log_adress;
				log_adress -= 0x04;

				// for??øΩ?øΩ?ÂÆπ Ê∏õÔøΩ???øΩ?øΩÂèØÂê¶Âà§??øΩ?øΩ? ‰∏çÂèØ„Å™„ÇâÔøΩ???øΩ?øΩÂ∫¶Êõ¥Êñ∞
				for(log_cmp = log_array - 1; log_cmp >= 0 ;log_cmp--) {

					if(log_cmp != log_array - 1) mm_buf = *(float*)log_adress;

					if(log_cmp == log_array - 1) {	// goalÂú∞ÁÇπ„ÅØEND_VELOCITY„Å®ÊØîÔøΩ?
						if( END_VELOCITY2 < PlanVelo2[log_cmp] - sqrtf( 2.0f * DECELERATION2 * END_DISTANCE2 ) ) PlanVelo2[log_cmp] = END_VELOCITY2 + sqrtf( 2.0f * DECELERATION2 * END_DISTANCE2 ) ;
					}
					else{	// 1Âå∫ÈñìÔøΩ???øΩ?øΩ??øΩ?øΩ???øΩ?øΩÈÄüÂ∫¶„Å®ÊØîÔøΩ?
						if( PlanVelo2[log_cmp + 1] < PlanVelo2[log_cmp] - sqrtf( 2.0f * DECELERATION2 * mm_buf ) ) PlanVelo2[log_cmp] = PlanVelo2[log_cmp + 1] + sqrtf( 2.0f * DECELERATION2 * mm_buf );
					}

					log_adress -= 0x08;
				}



				// for??øΩ?øΩ?ÂÆπ Ê∏õÔøΩ???øΩ?øΩÂå∫Èñì„Çí„Åô„Åπ„Å¶??øΩ?øΩ?‰ΩéÔøΩ???øΩ?øΩÂ∫¶„Å´ËêΩ„Å®„Åó„Å¶„Åî„ÇäÊäº??øΩ?øΩ?
				/*log_adress = end_log_adress;
				log_adress -= 0x04;

				for(log_cmp = log_array - 1; log_cmp >= 0 ;log_cmp--) {

					if(log_cmp != log_array - 1) mm_buf = *(float*)log_adress;

					if(log_cmp == log_array - 1) {	// goalÂú∞ÁÇπ„ÅØEND_VELOCITY„Å®ÊØîÔøΩ?

						velo_temp = PlanVelo[log_cmp];
						if( END_VELOCITY < PlanVelo[log_cmp] - sqrtf( 2.0f * DECELERATION * END_DISTANCE ) ) {
							PlanVelo[log_cmp] = MIN_VELOCITY;
						}
					}
					else{	// 1Âå∫ÈñìÔøΩ???øΩ?øΩ??øΩ?øΩ???øΩ?øΩÈÄüÂ∫¶„Å®ÊØîÔøΩ?

						if( velo_temp < PlanVelo[log_cmp] - sqrtf( 2.0f * DECELERATION * mm_buf ) ) {
							velo_temp = PlanVelo[log_cmp];
							PlanVelo[log_cmp] = MIN_VELOCITY;
						}
					}

					log_adress -= 0x08;
				}*/

				// for??øΩ?øΩ?ÂÆπ??øΩ?øΩ?Ë®àÁîª??øΩ?øΩ?„Éº„Çø‰øùÔøΩ?
				for(log_cmp = 0; log_cmp < log_array; log_cmp++) {
					FLASH_Write_Word_F(plan_velo_adress, PlanVelo[log_cmp]);
					plan_velo_adress += 0x04;
				}

				plan_velo_adress = start_adress_sector11;
				// for??øΩ?øΩ?ÂÆπ??øΩ?øΩ?Ë®àÁîª??øΩ?øΩ?„Éº„Çø‰øùÔøΩ?
				for(log_cmp = 0; log_cmp < log_array; log_cmp++) {
					FLASH_Write_Word_F(plan_velo_adress, PlanVelo2[log_cmp]);
					plan_velo_adress += 0x04;
				}

				main_pattern++;
				led_pattern(3);
				break;
			case 22:
				lcd_locate(0,0);
				lcd_print("_Finish_");
				lcd_locate(0,1);
				lcd_printf("%f",mileage((float)(enc_tim1_total + enc_tim8_total) / 2.0f));
				if(SW_UP == 0) {
					main_pattern = 0;
					second_trace_flag = 1;
					setup_mode = 0;
				}
				break;
			/*case 23:
				lcd_locate(0,0);
				lcd_print("End");
				lcd_locate(0,1);
				lcd_printf("T%7d",tim_buf);
				break;*/
			default:
				break;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 19;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 840;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
