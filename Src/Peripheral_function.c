#include "Peripheral_function.h"

uint16_t ADC1_Buff[ADC_DATA_BUFFR_SIZE];
uint16_t line_sen0;
uint16_t line_sen1;
uint16_t line_sen2;
uint16_t line_sen3;
uint16_t line_sen4;
uint16_t line_sen5;
uint16_t line_sen6;
uint16_t line_sen7;
uint16_t line_sen8;
uint16_t line_sen9;
uint16_t line_sen10;
uint16_t line_sen11;
uint16_t line_sen12;
uint16_t line_sen13;
uint16_t line_sen14;
uint16_t line_sen15;
uint16_t line_senLLL = 0;
uint16_t line_senLL = 0;
uint16_t line_senL = 0;
uint16_t line_senR = 0;
uint16_t line_senRR = 0;
uint16_t line_senRRR = 0;
char setup_mode = 0;
char sw_left;
char sw_right;
char sw_up;
char sw_down;
char sw_push;
char sw_tactile;
signed char operation_check;
int64_t enc_tim1_total;
int64_t enc_tim8_total;
//int64_t enc_tim1_cnt;
//int64_t enc_tim8_cnt;
int64_t enc_cnt;
int64_t enc_cnt2;
short cnt_sw;
unsigned char main_pattern = 0;
unsigned short volt_reg;
unsigned short current_reg;
float velR = 0.0f;
float velL = 0.0f;
int posR;
int posL;
char error_flag = 0;
int timer = 0;
uint16_t error_cnt;
float target_vel;
uint8_t maker_check;
char crossline_flag = 0;
unsigned char velocity_pattern = 0;
int encoder_event = 0;
uint8_t flash_flag = 0;
uint32_t log_adress;
int32_t enc_tim1_cnt_10ms;
int32_t enc_tim8_cnt_10ms;
uint32_t* flash_read_test;
uint16_t maker_cnt;
uint16_t maker_flag;
uint8_t calibration_flag = 0;
int64_t sum_zg = 0;
int16_t offset_zg = 0;
int16_t calibration_cnt;
float log_mm, log_zg;
float PlanVelo[6000];
int16_t log_array = 0;
uint32_t plan_velo_adress;
uint8_t second_trace_flag = 0;
int64_t enc_tim_total;
float mm_total = 0;
uint32_t log_check_adress;
uint8_t start_goal_flag = 0;
uint32_t maker_adress;
//float maker_distance_L[600];
//float maker_distance_R[600];
uint16_t maker_distance_cmp_lim;
float PlanVelo2[6000];
uint8_t second_trace_pattern;


# ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
# else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
# endif /*__GNUC__*/

PUTCHAR_PROTOTYPE {
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
 return ch;
}

void All_init(void) {
	IO_init();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	lcd_init();
	INA260_init();
	if( IMU_init() == 1 ) {
		lcd_locate(0,0);
		lcd_print("WHO_AM_I");
		lcd_locate(0,1);
		lcd_print("SUCCESS");
	}
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC1_Buff, 16);
	log_adress = start_adress_sector7;
	plan_velo_adress = start_adress_sector10;
}

void ESC_Calibration(void){	// 1必要なかった
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MAX);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MAX);
	HAL_Delay(2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
}

void IO_init(void) {
	MR_SET;
	ML_SET;
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);	//LEDdriver
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);	//LEDdriver
}

//センサー基板左から		AD12 AD13 AD0 AD1 AD2 AD3   AD4 AD5 AD6 AD7 AD14 AD15
//マーカー基板左から		AD9 AD8 	AD10 AD11
void ADval_get(void) {
	line_sen0  = ADC1_Buff[0];
	line_sen1  = ADC1_Buff[1];
	line_sen2  = ADC1_Buff[2];
	line_sen3  = ADC1_Buff[3];
	line_sen4  = ADC1_Buff[4];
	line_sen5  = ADC1_Buff[5];
	line_sen6  = ADC1_Buff[6];
	line_sen7  = ADC1_Buff[7];
	line_sen8  = ADC1_Buff[8];
	line_sen9  = ADC1_Buff[9];
	line_sen10 = ADC1_Buff[10];
	line_sen11 = ADC1_Buff[11];
	line_sen12 = ADC1_Buff[12];
	line_sen13 = ADC1_Buff[13];
	line_sen14 = ADC1_Buff[14];
	line_sen15 = ADC1_Buff[15];
}

//センサー基板左からLLL LL L R RR RRR
void ADval_sum(void) {
	//line_senLLL = line_sen12 + line_sen13;
	line_senLLL = line_sen12;
	line_senLL = line_sen0  + line_sen1;
	line_senL = line_sen2  + line_sen3;
	line_senR = line_sen4  + line_sen5;
	line_senRR = line_sen6  + line_sen7;
	//line_senRRR = line_sen14 + line_sen15;
	line_senRRR = line_sen15;
}

void CrossCheck(uint16_t crossthreshold){

	if(crossline_flag == 0 && line_senLL + line_senL + line_senR + line_senRR < crossthreshold ) {
		crossline_flag = 1;
		enc_cnt = 0;
	}

	if(crossline_flag == 1 && mileage((float)enc_cnt) >= 90){
		crossline_flag = 0;
	}
}

uint8_t StartGoalCheck(uint8_t makerval) {
	uint8_t ret = 0;

	if( mileage((float)enc_cnt2) >= 20 && start_goal_flag == 0 && ( makerval == 8 || makerval == 12)) {
		start_goal_flag = 1;
	}

	if( start_goal_flag == 1 ) {
		if(makerval == 0) {
			start_goal_flag = 0;
			ret = 1;
		}
		else if( (makerval &= 0x03) > 0 && (makerval &= 0x03) < 8) {
			start_goal_flag = 0;
			enc_cnt2 = 0;
		}
	}

	return ret;
}

void MakerCheck(uint8_t makerval){

	//uint8_t makercheckcnt;

	static uint8_t led = 0;
	int32_t maker_pulseL;
	int32_t maker_pulseR;
	int64_t maker_pulse_center;
	static uint16_t maker_preset = 0;
	static uint8_t maker_check_flag = 0;
	static uint8_t cmp_flag = 0;

	if((0 < makerval && makerval <= 3) && maker_check_flag == 0) {
		maker_check_flag = 1;
		enc_cnt = 0;
	}

	if((maker_check_flag == 1) && (mileage((float)enc_cnt) >= 6) && makerval == 0 ) {

		if(second_trace_flag == 1){

			maker_adress = start_adress_sector9 + maker_preset;
			cmp_flag = 1;
			led_pattern(led);

			while(cmp_flag){

				maker_pulseL = *(int32_t*)maker_adress;
				maker_adress += 0x04;
				if( isnan((float)maker_pulseL) != 0 ) cmp_flag = 0;
				maker_pulseR = *(int32_t*)maker_adress;
				maker_adress += 0x04;
				maker_pulse_center = (int64_t)(maker_pulseL + maker_pulseR) / 2;

				if((enc_tim_total > maker_pulse_center - 3000) && (enc_tim_total < maker_pulse_center + 3000)){ //2020_02_09
				//if((enc_tim_total + 3000 > maker_pulse_center) && (enc_tim_total - 3000 < maker_pulse_center)){
					enc_tim1_total = (int64_t)maker_pulseL;
					enc_tim8_total = (int64_t)maker_pulseR;
					maker_preset += 0x08;

					while( mm_total > mileage((float)maker_pulse_center) ){
						plan_velo_adress -= 0x04;
						log_adress -= 0x08;
						mm_total -= *(float*)log_adress;
						target_vel = *(float*)plan_velo_adress;
					}

					cmp_flag = 0;
					led++;
					if(led > 7) led = 0;
				}
				else if( maker_pulse_center > enc_tim_total + 8000) cmp_flag = 0;
				//else;

			}

		}
		else {
			/*FLASH_Write_DoubleWord(maker_adress,enc_tim1_total);
			maker_adress += 0x08;
			FLASH_Write_DoubleWord(maker_adress,enc_tim8_total);
			maker_adress += 0x08;*/

			FLASH_Write_Word_S(maker_adress,(int32_t)enc_tim1_total);
			maker_adress += 0x04;
			FLASH_Write_Word_S(maker_adress,(int32_t)enc_tim8_total);
			maker_adress += 0x04;

			/*FLASH_Write_Word_F(maker_adress,mileage(enc_tim1_total));
			maker_adress += 0x04;
			FLASH_Write_Word_F(maker_adress,mileage(enc_tim8_total));
			maker_adress += 0x04;*/
		}

		maker_check_flag = 0;
	}

}

/*void MakerCheck(uint8_t makerval){

	float distance_now;
	static uint16_t pre_maker_cnt = 0;
	float maker_distance;
	uint8_t maker_cmp_flag = 1;

	if(crossline_flag == 0) {

		if(maker_flag == 0 && (makerval == 1 || makerval == 2 || makerval == 3)) {
			maker_flag = 1;

			enc_cnt = 0;
		}

		if(maker_flag == 1 && mileage((float)enc_cnt) >= 6 && makerval == 0) {

			//maker_cnt++;
			maker_flag = 0;
			if( second_trace_flag == 0 ){
				FLASH_Write_Word_F(maker_adress,mileage(enc_tim1_total));
				maker_adress += 0x04;
				FLASH_Write_Word_F(maker_adress,mileage(enc_tim8_total));
				maker_adress += 0x04;
			}
			else if( second_trace_flag == 1 ){
				maker_cnt = 0;
				maker_cnt = pre_maker_cnt + 1;
				distance_now = mileage( enc_tim_total);
				while( maker_cmp_flag ){

					maker_distance = (maker_distance_L[maker_cnt] + maker_distance_R[maker_cnt]) / 2.0f;
					if( ( distance_now - 25.0f < maker_distance ) && ( maker_distance < distance_now + 25.0f) ){
						led_pattern(2);
						enc_tim1_total = maker_distance_L[maker_cnt];
						enc_tim8_total = maker_distance_R[maker_cnt];
						pre_maker_cnt = maker_cnt;
						maker_cmp_flag = 0;
					}

					maker_cnt++;
					if(maker_cnt >= maker_distance_cmp_lim ) maker_cmp_flag = 0;
				}

			}

			log_mm = mileage((float)(enc_tim1_cnt_10ms + enc_tim8_cnt_10ms) / 2.0f);

			FLASH_Write_Word_F(log_adress,log_zg);
			log_adress += 0x04;
			FLASH_Write_Word_F(log_adress,log_mm);
			log_adress += 0x04;

			log_zg = 0;
			enc_tim1_cnt_10ms = 0;
			enc_tim8_cnt_10ms = 0;

			read_zg_data();
					log_zg = (float)zg / (MAXDATA_RANGE / GYRO_RANGE);
					enc_cnt_10ms = (enc_tim1_cnt_10ms + enc_tim8_cnt_10ms) / 2;
					log_mm = mileage((float)enc_cnt_10ms);
					FLASH_Write_Word_F(log_adress,log_zg);
					log_adress += 0x04;
					FLASH_Write_Word_F(log_adress,log_mm);
					log_adress += 0x04;
					enc_tim1_cnt_10ms = 0;
					enc_tim8_cnt_10ms = 0;
		}

	}

}*/

uint8_t MakerSenTh(uint16_t makerthreshold) {
	uint8_t maker = 0;

	if(crossline_flag == 0){
		if(line_sen9 < makerthreshold) maker |= 0x01;
		if(line_sen8 < makerthreshold) maker |= 0x02;
		if(line_sen10 < makerthreshold) maker |= 0x04;
		if(line_sen11 < makerthreshold) maker |= 0x08;
	}
	
	return maker;
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {	//AD変換完了時に呼び出される関数
	
}*/

void Calculation_offset_zg(){
	offset_zg = sum_zg / calibration_cnt;
}

float Velo_Spline_Curve(float curvature) {
	float velo_spline;

	velo_spline = 0.00000243536328504599f * powf(curvature, 3) + (-0.00768048107979400f) * powf(curvature, 2) + 8.55442953186553f * curvature + 154.785382404022f;
	//radius = [ 0, 100, 150, 200, 250, 300, 400, 500, 750, 1000, 1500, 2000];
	//vel = [0, 1200, 1200, 1500, 1800, 2200, 2500, 3000, 3000, 3500, 4000, 6000];

	//velo_spline = 0.000000958783462728946f * powf(curvature, 3) + (-0.00361845365313069f) * powf(curvature, 2) + 5.28713093017124f * curvature + 235.903467671659f;
	/*if( 300 <= curvature && curvature < 500) velo_spline = 2000.0f;
	if( 500 <= curvature && curvature < 1000) velo_spline = 3000.0f;
	if( 1000 <= curvature && curvature < 1500) velo_spline = 4000.0f;
	if( 1500 <= curvature ) velo_spline = 5000.0f;*/
	//if(velo_spline >= MAX_VELOCITY) velo_spline = MAX_VELOCITY;
	//else if(velo_spline < MIN_VELOCITY) velo_spline = MIN_VELOCITY;
	//else if(velo_spline <= 2200 ) velo_spline = MIN_VELOCITY;
	return velo_spline;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	uint64_t enc_cnt_10ms;

	if(htim->Instance == htim6.Instance){

		getEncoder();
		ADval_get();
		ADval_sum();

		read_zg_data();
		read_xa_data();

		if(calibration_flag == 1) {
			sum_zg += zg;
			calibration_cnt++;
		}

		if(main_pattern == 0) {
			cnt_sw++;
			maker_check = MakerSenTh(MAKERTHRESHOLD);
			if(cnt_sw >= 50) {
				IOstate_get();
				cnt_sw = 0;
			}
		}
		else if(main_pattern>=10 && main_pattern<=19) {

			if(main_pattern == 13 && second_trace_flag == 1){
				while( mm_total < mileage((float)enc_tim_total ) ) {

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
					led_pattern(4);
					enc_cnt = 0;
					main_pattern++;
					timer = 0;
				}
			}
			else if(main_pattern==14){
				if (mileage((float)enc_cnt) >= 400) {
					target_vel = 0.0f;
					led_pattern(7);
					main_pattern = 20;
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				}
			}


			ErrorCheck(ERRORCHECK);
			timer++;
			CrossCheck(CROSSCHECK);
			maker_check = MakerSenTh(MAKERTHRESHOLD);//400 700
			MakerCheck(maker_check);
			posPID();
			velPID(target_vel);
			MotorCtrl((short)(order_velR + order_posR), (short)(order_velL + order_posL));
			//MotorCtrl((short)(order_velR), (short)(order_velL));
			//MotorCtrl((short)(order_posR), (short)(order_posL));
		}

	}

	if((htim->Instance == htim7.Instance) && (flash_flag == 1)) {
		//MakerCheck(maker_check);
		//log_zg += ((float)(zg - offset_zg) / 16.4f) * 0.01f;

		enc_cnt_10ms = (enc_tim1_cnt_10ms + enc_tim8_cnt_10ms) / 2;
		log_mm = mileage((float)enc_cnt_10ms);

		read_zg_data();
		log_zg = ((float)(zg - offset_zg) / 16.4f) * 0.01f;	//θ算出
		log_zg = fabsf(log_zg);// 絶対値
		if( crossline_flag == 1 ) log_zg = 0;

		if( log_zg == 0 ) PlanVelo[log_array] = 10000;	// θが0の場合は曲率半径は10000とする
		else PlanVelo[log_array] = log_mm / ( 2.0f * PI * ( log_zg / 360) );
		log_array++;
		//log_zg = (float)zg / 16.4f;
		FLASH_Write_Word_F(log_adress,log_zg); // 曲率半径
		log_adress += 0x04;

		FLASH_Write_Word_F(log_adress,log_mm); // 距離
		log_adress += 0x04;

		enc_tim1_cnt_10ms = 0;
		enc_tim8_cnt_10ms = 0;
	}

}

void led_pattern(char led) {
	switch(led) {
		case 0:	//無
			LED_R_SET;
			LED_G_SET;
			LED_B_SET;
			break;
		case 1:	//赤
			LED_R_RESET;
			LED_G_SET;
			LED_B_SET;
			break;
		case 2:	//緑
			LED_R_SET;
			LED_G_RESET;
			LED_B_SET;
			break;
		case 3:	//青
			LED_R_SET;
			LED_G_SET;
			LED_B_RESET;
			break;
		case 4:	//黄
			LED_R_RESET;
			LED_G_RESET;
			LED_B_SET;
			break;
		case 5:	//水
			LED_R_SET;
			LED_G_RESET;
			LED_B_RESET;
			break;
		case 6:	//紫
			LED_R_RESET;
			LED_G_SET;
			LED_B_RESET;
			break;
		case 7:	//白
			LED_R_RESET;
			LED_G_RESET;
			LED_B_RESET;
			break;
		default://無
			LED_R_SET;
			LED_G_SET;
			LED_B_SET;
			break;
	}
}

void IOstate_get(void) {
	
	if(SW_LEFT == 0 && sw_left == 0) sw_left = 1;
	/*else if(SW_LEFT == 0 && sw_left == 1) ;
	else if(SW_LEFT == 1 && sw_left == 1) {
		setup_mode++;
		sw_left = 0;
	}*/
	else sw_left = 0;
	
	if(SW_RIGHT == 0) sw_right = 1;
	else sw_right = 0;
	
	if(SW_UP == 0) sw_up = 1;
	//else if(SW_UP == 0 && sw_up == 1) ;
	else if(SW_UP == 1 && sw_up == 1) {
		operation_check++;
		sw_up = 0;
	}
	else sw_up = 0;
	
	if(SW_DOWN == 0) sw_down = 1;
	//else if(SW_DOWN == 0 && sw_down == 1) ;
	else if(SW_DOWN == 1 && sw_down == 1) {
		operation_check--;
		sw_down = 0;
	}
	else sw_down = 0;
	
	if(SW_PUSH == 0) sw_push = 1;
	else sw_push = 0;
	
	if(SW_TACTILE == 0) sw_tactile = 1;
	//else if(SW_TACTILE == 0 && sw_tactile == 1) ;
	else if(SW_TACTILE == 1 && sw_tactile == 1) {
		setup_mode++;
		sw_tactile = 0;
	}
	else sw_tactile = 0;
	
}

void getEncoder(void) {
	
	int16_t enc_tim1_ms;
	int16_t enc_tim8_ms;
	
	enc_tim1_ms = TIM1 -> CNT;
	enc_tim8_ms = TIM8 -> CNT;
	
	TIM1 -> CNT = 0;
	TIM8 -> CNT = 0;
	
	enc_tim1_total += enc_tim1_ms;
	enc_tim8_total += enc_tim8_ms;
	enc_tim_total = (enc_tim1_total + enc_tim8_total) / 2;

	enc_cnt += ((enc_tim1_ms + enc_tim8_ms) / 2.0f);
	enc_cnt2 += ((enc_tim1_ms + enc_tim8_ms) / 2.0f);
	//enc_tim1_cnt += enc_tim1_ms;
	//enc_tim8_cnt += enc_tim8_ms;
	
	enc_tim1_cnt_10ms += enc_tim1_ms;
	enc_tim8_cnt_10ms += enc_tim8_ms;

	velR = (float)enc_tim8_ms * ENC_PULSE_MM * 1000.0f;
	velL = (float)enc_tim1_ms * ENC_PULSE_MM * 1000.0f;
	
}

float mileage(float mm) {
	return mm * ENC_PULSE_MM;
}

void MotorCtrl(short motorR, short motorL) {
	
	short pwmL_out,pwmR_out;
	
	if(motorR >= 0) {
		pwmR_out = motorR;
		MR_SET;
	}
	else {
		pwmR_out = motorR * (-1);
		MR_RESET;
	}
	
	if(motorL >= 0) {
		pwmL_out = motorL;
		ML_SET;
	}
	else {
		pwmL_out = motorL * (-1);
		ML_RESET;
	}
	
	if(pwmR_out > 840) pwmR_out = 839;
	if(pwmL_out > 840) pwmL_out = 839;

	if(error_flag == 1) {
		pwmR_out = 0;
		pwmL_out = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwmR_out);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwmL_out);
	
}

void setup(void) {

	float read_log_zg, read_log_mm;

	//if(SW_PUSH == 0) operation_check++;
	//if(SW_DOWN == 0) operation_check--;
	
	if(operation_check <  0) operation_check = 16;
	if(operation_check >=17) operation_check = 0;

	if(setup_mode >= 8) setup_mode = 0;
	//if(sw_tactile == 0) setup_mode++;
	
	led_pattern(setup_mode);
	
	switch(setup_mode) {
		case 0:	//AD確認
			
			switch(operation_check) {
				case 0:
					lcd_locate(0,0);
					lcd_printf("%4d AD0",line_sen0);
					lcd_locate(0,1);
					lcd_printf("%4d AD1",line_sen1);
					break;
				case 1:
					lcd_locate(0,0);
					lcd_printf("%4d AD2",line_sen2);
					lcd_locate(0,1);
					lcd_printf("%4d AD3",line_sen3);
					break;
				case 2:
					lcd_locate(0,0);
					lcd_printf("%4d AD4",line_sen4);
					lcd_locate(0,1);
					lcd_printf("%4d AD5",line_sen5);
					break;
				case 3:
					lcd_locate(0,0);
					lcd_printf("%4d AD6",line_sen6);
					lcd_locate(0,1);
					lcd_printf("%4d AD7",line_sen7);
					break;
				case 4:
					lcd_locate(0,0);
					lcd_printf("%4d AD8",line_sen8);
					lcd_locate(0,1);
					lcd_printf("%4d AD9",line_sen9);
				break;
				case 5:
					lcd_locate(0,0);
					lcd_printf("%4dAD10",line_sen10);
					lcd_locate(0,1);
					lcd_printf("%4dAD11",line_sen11);
					break;
				case 6:
					lcd_locate(0,0);
					lcd_printf("%4dAD12",line_sen12);
					lcd_locate(0,1);
					lcd_printf("%4dAD13",line_sen13);
					break;
				case 7:
					lcd_locate(0,0);
					lcd_printf("%4dAD14",line_sen14);
					lcd_locate(0,1);
					lcd_printf("%4dAD15",line_sen15);
					break;
				case 8:
					lcd_locate(0,0);
					lcd_printf("XG%6x",xg);
					lcd_locate(0,1);
					lcd_printf("YG%6x",yg);
					break;
				case 9:
					lcd_locate(0,0);
					lcd_printf("ZG%6x",zg);
					lcd_locate(0,1);
					lcd_printf("XA%6x",xa);
					break;
				case 10:
					lcd_locate(0,0);
					lcd_printf("YA%6x",ya);
					lcd_locate(0,1);
					lcd_printf("ZA%6x",za);
					break;
				case 11:
					lcd_locate(0,0);
					lcd_print("error_th");
					lcd_locate(0,1);
					lcd_printf("%8d",line_senLLL + line_senLL + line_senL + line_senR + line_senRR + line_senRRR);
					break;
				case 12:
					lcd_locate(0,0);
					lcd_print("cross_th");
					lcd_locate(0,1);
					lcd_printf("%8d", line_senLL + line_senL + line_senR + line_senRR);
					break;
				case 13:
					lcd_locate(0,0);
					lcd_print("maker_th");
					lcd_locate(0,1);
					lcd_printf("%8d", maker_check);
					break;
				case 14:
					lcd_locate(0,0);
					lcd_printf("%f", mileage((float)enc_tim1_total));
					lcd_locate(0,1);
					lcd_printf("%f", mileage((float)enc_tim8_total));
					break;
				case 15:
					lcd_locate(0,0);
					lcd_printf("%f", ((float)line_senLLL * 1.65f) + ((float)line_senLL * 1.2f) + (float)line_senL);
					lcd_locate(0,1);
					lcd_printf("%f", (float)line_senR + ((float)line_senRR * 1.2f) + ((float)line_senRRR) * 1.65f);
					break;
				case 16:
					lcd_locate(0,0);
					lcd_print("Voltage_");
					lcd_locate(0,1);
					volt_reg = INA260_read(0x02);
					lcd_printf("   %1.2fV",(float)volt_reg*0.00125f+0.05f);//0.05f←オフセット
					break;
				default:
					break;
			}
			
			break;
		case 1:
			if( SW_PUSH == 0 ) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2116);	//	1763(ESC_MIN) + 17.64 * 20
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2116);
			}
			else {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
			}
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
			if( SW_PUSH == 0 ) {
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 400);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 400);
				MR_SET;
				ML_SET;
			}
			else {
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				MR_SET;
				ML_SET;
			}
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

			lcd_locate(0,0);
			lcd_print("kill7&10");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			/*if( SW_PUSH == 0 ) FLASH_Erease7();
			if( SW_RIGHT == 0 ) FLASH_Erease9();
			if( SW_DOWN == 0 ) FLASH_Erease10();
			if( SW_UP == 0 ) FLASH_Erease11();*/

			if( SW_PUSH == 0 ) {
				FLASH_EreaseSector(FLASH_SECTOR_7);
				FLASH_EreaseSector(FLASH_SECTOR_9);
				FLASH_EreaseSector(FLASH_SECTOR_10);
				FLASH_EreaseSector(FLASH_SECTOR_11);
			}

			break;
		case 4:
			lcd_locate(0,0);
			lcd_print("_log_tx_");
			lcd_locate(0,1);
			lcd_print("________");

			if(SW_PUSH == 0) {
				/*read_log_zg = *(float*)log_adress;
				log_adress += 0x04;
				read_log_mm = *(float*)log_adress;
				log_adress += 0x04;
				//isnan()　NaN(非数)かどうかの判定
				if(isnan(read_log_zg) == 0) printf("%f %f\r\n",read_log_zg,read_log_mm);*/

				read_log_mm = *(float*)plan_velo_adress;
				plan_velo_adress += 0x04;
				if(isnan(read_log_mm) == 0) printf("%f\r\n",read_log_mm);
			}

			break;
		case 5:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 1 ");
			if(SW_PUSH == 0) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				velocity_pattern = 1;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 6:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 2 ");
			if(SW_PUSH == 0) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				log_check_adress = start_adress_sector10;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 1;
				}
				else velocity_pattern = 2;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 7:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 3 ");
			if(SW_PUSH == 0) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				log_check_adress = start_adress_sector11;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 2;
				}
				else velocity_pattern = 3;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		default:
			break;
	}
}
