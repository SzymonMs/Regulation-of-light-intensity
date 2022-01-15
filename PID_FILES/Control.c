/*
 * Control.c
 *
 *  Created on: Jan 15, 2022
 *      Author: szymo
 */

#include "main.h"
#include "Control.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
extern luxint;
extern set_value;
extern duty;
void PID_regulator(struct PID pid)
{
	luxint=bh1750_read();
	float error = set_value - luxint;
	float u;

	float u_P =pid.Kp * error;

	float integral = (error + pid.prev_error + pid.prev_int);
	float u_I = pid.Ki * pid.Tp/2 * integral;
	pid.prev_int = integral;

	float derivative=(error-pid.prev_error)/pid.Tp;
	//float u_D=pid.Td*derivative;
	float u_D=pid.Td*pid.N/(1+pid.N*(1/derivative));
	pid.prev_error = error;

	u = u_P + u_I+u_D;

	duty = (uint16_t)u;

	if (duty>999)
	{
		duty = 999;
	}
	htim3.Instance->CCR1 = duty;
}
void step(uint8_t duty)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,duty*10);
}
