/*
 * stm32f072Tx_gpio_driver.h
 *
 *  Created on: 28.04.2019
 *      Author: Pawel
 *
 *      Driver specific data
 */



#ifndef INC_STM32F072TX_GPIO_DRIVER_H_
#define INC_STM32F072TX_GPIO_DRIVER_H_

#include "stm32f072xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;									/*		This holds possible @GPIO_PIN_NUMBERS		*/
	uint8_t GPIO_PinMode;									/*		This holds possible @GPIO_PIN_MODES			*/
	uint8_t GPIO_PinSpeed;									/*		This holds possible @GPIO_PIN_SPEEDS		*/
	uint8_t GPIO_PinPuPdControl;							/*		This holds possible @GPIO_PUSHPULL_MODES	*/
	uint8_t GPIO_PinOPType;									/*		This holds possible @GPIO_OUTPUT_TYPE		*/
	uint8_t GPIO_PinAltFunMode;								/*		This holds possible @GPIO_ALTERNATE_FUNCTIONS*/
}GPIO_PinConfig_t;



/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIOs possible pin numbers
 */

#define GPIO_PIN_NO0		0
#define GPIO_PIN_NO1		1
#define GPIO_PIN_NO2		2
#define GPIO_PIN_NO3		3
#define GPIO_PIN_NO4		4
#define GPIO_PIN_NO5		5
#define GPIO_PIN_NO6		6
#define GPIO_PIN_NO7		7
#define GPIO_PIN_NO8		8
#define GPIO_PIN_NO9		9
#define GPIO_PIN_NO10		10
#define GPIO_PIN_NO11		11
#define GPIO_PIN_NO12		12
#define GPIO_PIN_NO13		13
#define GPIO_PIN_NO14		14
#define GPIO_PIN_NO15		15




/*
 * @GPIO_PIN_MODES
 * GPIOs possible modes
 */

#define GPIO_MODE_INPT		0
#define GPIO_MODE_OUTP		1
#define GPIO_MODE_ALTFUN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6



/*
 * @GPIO_OUTPUT_TYPE
 * GPIOs possible port output types
 */

#define GPIO_OUTP_TYPE_PP	0			//PUSH-PULL
#define GPIO_OUTP_TYPE_OD	1			//OPEN DRAIN



/*
 * @GPIO_PIN_SPEEDS
 * GPIOs possible output speeds
 */

#define GPIO_OUTP_SPEED_LOW	0
#define GPIO_OUTP_SPEED_MED	1
#define GPIO_OUTP_SPEED_HI	3


/*
 * @GPIO_PUSHPULL_MODES
 * GPIOs pull-up/pull-down
 */

#define GPIO_PIN_NOPUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define GPIO_PIN_RESERVED	3





/****************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 ****************************************************************************/

/*
 * Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);



/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);  //IRQGrouping - mo¿na doda
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);



#endif /* INC_STM32F072TX_GPIO_DRIVER_H_ */
