/*
 * 002led_button.c
 *
 *  Created on: 05.05.2019
 *      Author: Pawel
 */

#include "stm32f072xx.h"

void delay()
{
	for(int i =0; i<250000; ++i);
}



int main(void)
{
	GPIO_Handle_t gpioLed;
	GPIO_Handle_t gpioButton;
	//RCC_RegDef_t pRCC;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO9;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPT;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO0;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	//while(!);
	GPIO_Init(&gpioLed);
	while (1)
	{
		//if( gpioButton.pGPIOx->IDR &= 0x1) //tak zrobi³em ja - tak samo dzia³a
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO9);

		}
		//else GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO9, DISABLE);
	}
}
