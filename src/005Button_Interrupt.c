/*
 * 005Button_Interrupt.c
 *
 *  Created on: 06.05.2019
 *      Author: Pawel
 */

#include "stm32f072Tx_gpio_driver.h"
#include "stm32f072xx.h"
#include <string.h>

#define LED_Count 4U

void delay()
{
	for (uint16_t i = 0; i<60000; ++i);
}





int main(void)
{
	GPIO_Handle_t gpioLedG, gpioLedR, gpioLedO, gpioLedB;
	GPIO_Handle_t gpioButton;

	memset(&gpioLedG, 0, sizeof(gpioLedG));
	memset(&gpioLedB, 0, sizeof(gpioLedB));
	memset(&gpioLedR, 0, sizeof(gpioLedR));
	memset(&gpioLedO, 0, sizeof(gpioLedO));
	memset(&gpioButton, 0, sizeof(gpioButton));



	gpioLedG.pGPIOx = GPIOC;
	gpioLedG.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO9;
	gpioLedG.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLedG.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLedG.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLedG.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	gpioLedR.pGPIOx = GPIOC;
	gpioLedR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
	gpioLedR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLedR.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLedR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLedR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	gpioLedO.pGPIOx = GPIOC;
	gpioLedO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO8;
	gpioLedO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLedO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLedO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLedO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	gpioLedB.pGPIOx = GPIOC;
	gpioLedB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO7;
	gpioLedB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	gpioLedB.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	gpioLedB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_LOW;
	gpioLedB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);
	GPIO_Init(&gpioLedG);
	GPIO_Init(&gpioLedB);
	GPIO_Init(&gpioLedO);
	GPIO_Init(&gpioLedR);
	//IRQ Configurations
	//GPIO_IRQPriorityConfig(, IRQ_NO_EXTI0_1);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);


	GPIO_RegDef_t* LEDS[]={gpioLedG.pGPIOx, gpioLedR.pGPIOx, gpioLedB.pGPIOx, gpioLedO.pGPIOx}; //just in case GPIO ports are different
	uint8_t LED_queue[]= {GPIO_PIN_NO9, GPIO_PIN_NO6, GPIO_PIN_NO7, GPIO_PIN_NO8};

	while(1)
	{
		for(uint8_t i = 0; i < (uint8_t)LED_Count; ++i)
		{
			GPIO_ToggleOutputPin(LEDS[i], LED_queue[i]);
			delay();
		}
	}
}



void EXTI0_1_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO0);
	//GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO9);
}
