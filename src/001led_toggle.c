/*
 * 001led_toggle.c
 *
 *  Created on: 01.05.2019
 *      Author: Pawel
 */

#include "stm32f072xx.h"

void delay()
{
	for(int i =0; i<50000; ++i);
}



int main(void)
{
	GPIO_Handle_t gpioLed;
	//RCC_RegDef_t pRCC;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO9;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	//while(!);
	GPIO_Init(&gpioLed);
	while (1)
	{
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO9);
			delay();
	}
}
