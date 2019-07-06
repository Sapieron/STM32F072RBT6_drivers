/*
 * stm32f072Tx_gpio_driver.c
 *
 *  Created on: 28.04.2019
 *      Author: Pawel
 */


#include "stm32f072Tx_gpio_driver.h"



/*
 * Peripheral clock setup
 */

/******************************************************************
 * @fn			- GPIO_PeriClock_Control
 *
 * @brief		- This function enables or disables peripheral clock
 *
 * @param[in]	- Base address of GPIOx peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 */




void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
	}
	else 																		//disable clock
	{
		if(pGPIOx == GPIOA)
			GPIOA_REG_RESET();
		else if(pGPIOx == GPIOB)
			GPIOB_REG_RESET();
		else if(pGPIOx == GPIOC)
			GPIOC_REG_RESET();
		else if(pGPIOx == GPIOD)
			GPIOD_REG_RESET();
		else if(pGPIOx == GPIOE)
			GPIOE_REG_RESET();
		else if(pGPIOx == GPIOF)
			GPIOF_REG_RESET();
	}

}



/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//0 . Configure the clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);



	//1 . Configure the mode of gpio pin
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //if the chosen mode is smaller or equal to 3, than it behaves as non-ISR handler, check notebook
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //pGPIOHandle address is taken, then we access it's module GPIO_PinConfig, next we take value of GPIO_PinMode which is a part of this module
			pGPIOHandle->pGPIOx->MODER &= ~(0x3  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER |= temp;
		}
		else
		{
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}


			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] |= (GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) << (temp2 *4)); //EXTI configuration bits


			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		temp = 0;

		//2 . configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp=0;

		//3. configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;


		//4. configure the optype
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;


		//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
		{
			uint8_t temp1, temp2;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2);
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4 * temp2);
		}


}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

			if(pGPIOx == GPIOA)
				GPIOA_REG_RESET();
			else if(pGPIOx == GPIOB)
				GPIOB_REG_RESET();
			else if(pGPIOx == GPIOC)
				GPIOC_REG_RESET();
			else if(pGPIOx == GPIOD)
				GPIOD_REG_RESET();
			else if(pGPIOx == GPIOE)
				GPIOE_REG_RESET();
			else if(pGPIOx == GPIOF)
				GPIOF_REG_RESET();
}



/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == 1)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &=~(1 << PinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)  //IRQGrouping - mo¿na doda
{
	if(EnorDi==ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*(NVIC_ISER_0) |= (1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			//TODO - no need as in M0 architecture there are no more NVIC registers
		}
		else if(IRQNumber > 63 && IRQNumber <=95)
		{
			//TODO - no need as in M0 architecture there are no more NVIC registers
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*(NVIC_ICER_0) |= (1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			//TODO - no need as in M0 architecture there are no more NVIC registers
		}
		else if(IRQNumber > 63 && IRQNumber <=95)
		{
			//TODO - no need as in M0 architecture there are no more NVIC registers
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t IRQReg = IRQNumber / 4;
	uint8_t IRQByteSection = IRQNumber % 4;
	uint32_t shiftBy = (IRQByteSection * 8) + (8 - AMMOUNT_OF_BITS_IMPLEMENTED );

	*(NVIC_PR_BASE_ADDR + IRQReg ) |= (IRQPriority << shiftBy ) ;
}




void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
