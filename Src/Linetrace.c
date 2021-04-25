#include "Linetrace.h"

float order_posR = 0.0f;
float order_posL = 0.0f;
float order_velR = 0.0f;
float order_velL = 0.0f;

void ErrorCheck(uint16_t errorthreshold) {
	if((line_senLLL + line_senLL + line_senL + line_senR + line_senRR + line_senRRR) > errorthreshold) {
		error_cnt++;
		if(error_cnt >= 5) {
			error_flag = 1;
			main_pattern = 20;
			target_vel = 0.0f;
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
		}
		if(error_cnt > 60000) error_cnt = 1000;
	}
}

void posPID(void) {

	float p_pos, d_pos;
	static float i_pos;
	float kp_pos = 0.10f, ki_pos = 0.004f, kd_pos = 0.008f;
	//float kp_pos = 0.10f, ki_pos = 0.004f, kd_pos = 0.008f;
	//float kp_pos = 0.09f, ki_pos = 0.002f, kd_pos = 0.006f;
	//float dt_pos = 0.001f;
	static float def_pos[] = {0.0f, 0.0f};

	def_pos[0] = ( ((float)line_senLLL * 1.6f) + ((float)line_senLL * 1.25f) + (float)line_senL + LINE_OFFSET_L) - ((float)line_senR + ((float)line_senRR * 1.25f) + ((float)line_senRRR * 1.6f)); //1.25 1.6

	p_pos = kp_pos * def_pos[0]; //P制御
	i_pos += ki_pos * def_pos[0] * DELTA_T; //I制御
	d_pos = kd_pos * (def_pos[0] - def_pos[1]) / DELTA_T; //D制御

	order_posR = p_pos + i_pos + d_pos;
	order_posL = -(p_pos + i_pos + d_pos);

	def_pos[1] = def_pos[0];

}

/*void velPID(float target) {
	float p_vel, kp_vel = 2.8f, ki_vel = 50.0f, vel_center;//2.8 50
	static float i_vel, def_vel;

	vel_center = (velR + velL) / 2.0f;
	def_vel = vel_center - target;

	p_vel = kp_vel * def_vel;
	i_vel += ki_vel * def_vel * DELTA_T;

	order_velR = p_vel + i_vel;
	order_velL = p_vel + i_vel;
}*/

void velPID(float target) {
	float p_vel, kp_vel = 2.8f, ki_vel = 50.0f;	//2.8 50
	float vel_center, filter_vel_center, acceleration_imu;
	static float i_vel, def_vel, last_vel_center;

	vel_center = (velR + velL) / 2.0f;
	acceleration_imu = (float)xa / 16384.0f;
	filter_vel_center = ComplementaryFilter(acceleration_imu, vel_center, 0.65f, last_vel_center);
	last_vel_center = filter_vel_center;

	def_vel = filter_vel_center - target;

	p_vel = kp_vel * def_vel;
	i_vel += ki_vel * def_vel * DELTA_T;

	order_velR = p_vel + i_vel;
	order_velL = p_vel + i_vel;
}

/*
#define	MASS			0.110f	//	[kg]	機体質量
#define RADIUS_TIRE		0.011f	//	[m]		タイヤ半径
#define GEAR_RATIO		2.72f				ギヤ比
#define	Kt				0.00352f//	[Nm/A]	トルク定数
#define	Ke				0.00352f//	[V/rpm]	逆起電力定数　間違い(単位)
#define R_MOTOR			2.9f	//	[Ω]		モータ端子間抵抗
*/

/*
 * Kt = Ke
 * 1分間あたりのパルス数 = 1msでカウントしたパルス * 60[s] / 0.001[s]
 * モータ1回転のパルス数  (512*4逓倍)pulse = 2048
 * 回転数(rpm) = 1分間あたりのパルス数 / モータ1回転のパルス数
 * この計算間違ってた
 */

/*
void AccelFeedForward(float accel) {
	float T_tire, T_motor, I_motor, V_motor, V_induced, FF_duty;

	T_tire = MASS * (accel / 2.0f) * RADIUS_TIRE;
	T_motor = T_tire / GEAR_RATIO;
	I_motor = T_motor / Kt;
	V_induced = Ke * rpm;
	V_motor = R_MOTOR * I_motor + V_induced;
	FF_duty = V_motor / V_batt;
}
*/

/*void velPID(float targetR, float targetL) {
	float p_velR, p_velL;
	static float i_velR, i_velL;
	float kp_velL = 2.8f, kp_velR = 2.8f;
  float ki_velL = 50.0f, ki_velR = 50.0f;
	// float d_velR, d_velL, kd_velR = 0.0f, kd_velL = 0.0f
	//static float def_velR[] = {0.0f, 0.0f};
	//static float def_velL[] = {0.0f, 0.0f};
	static float def_velR = 0.0f;
	static float def_velL = 0.0f;

	def_velR = velR - targetR;
	def_velL = velL - targetL;

	p_velR = kp_velR * def_velR;
	i_velR += ki_velR * def_velR * DELTA_T;
	//d_velR = kd_velR * (def_velR[0] - def_velR[1]) / DELTA_T;

	p_velL = kp_velL * def_velL;
	i_velL += ki_velL * def_velL * DELTA_T;
	//d_velL = kd_velL * (def_velL[0] - def_velL[1]) / DELTA_T;

	order_velR = p_velR + i_velR;
	order_velL = p_velL + i_velL;

	//def_velR[1] = def_velR[0];
	//def_velL[1] = def_velL[0];

}*/

/*void velPID(float targetR, float targetL) {
	float p_velR, i_velR, d_velR, p_velL, i_velL, d_velL;
	float kp_velR = 10.0f, ki_velR = 0.0f, kd_velR = 0.0f;
	float kp_velL = 10.0f, ki_velL = 0.0f, kd_velL = 0.0f;
	float dt_vel = 0.001f;
	static float def_velR[] = {0.0f, 0.0f, 0.0f};
	static float def_velL[] = {0.0f, 0.0f, 0.0f};

	def_velR[0] = targetR - velR;
	def_velL[0] = targetL - velL;

	p_velR = kp_velR * (def_velR[0] - def_velR[1]);
	i_velR = ki_velR * def_velR[0];
	d_velR = kd_velR * (def_velR[0] - 2.0f * def_velR[1] + def_velR[2]);

	p_velL = kp_velL * (def_velL[0] - def_velL[1]);
	i_velL = ki_velL * def_velL[0];
	d_velL = kd_velL * (def_velL[0] - 2.0f * def_velL[1] + def_velL[2]);

	order_velR += p_velR + i_velR + d_velR;
	order_velL += p_velL + i_velL + d_velL;

	def_velR[2] = def_velR[1];
	def_velR[1] = def_velR[0];

	def_velL[2] = def_velL[1];
	def_velL[1] = def_velL[0];
	
}*/
