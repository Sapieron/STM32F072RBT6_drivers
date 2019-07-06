/*
 * stm32f072xx.h
 *
 *  Created on: 27.04.2019
 *      Author: Pawel
 *
 *      MCU specific data
 */

#include <stdint.h>

#ifndef INC_STM32F072XX_H_
#define INC_STM32F072XX_H_

/******************************Processor specific details****************************/
/*
 * NOTE: These addresses are taken from ARM Cortex M0 generic user guide
*/

/*
 * Arm Cortex Mx processor NVIC ISERx register Addresses
 */

#define NVIC_ISER_0		((volatile uint32_t*)0xE000E100)	/*Interrupt Set Enable Register*/



/*
 * Arm Cortex Mx processor NVIC ICERx register Addresses
 */

#define NVIC_ICER_0		((volatile uint32_t*)0xE000E180)	/*Interrupt Clear Enable Register*/

/*
 * ARM Cortex M0 Priority register address calculation
 */

#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)	//PR stands for priority registers

#define AMMOUNT_OF_BITS_IMPLEMENTED	2		//This acts

/*
 * base addresses of Flash and SRAM
 */
#define FLASH_BASEADDR							0x08000000U
#define SRAM1_BASEADDR							0x20000000U
#define SRAM_BASEADDR							SRAM1_BASEADDR
#define ROM_BASEADDR							0x1FFFC800U



/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APBPERIPH_BASEADDR						PERIPH_BASEADDR
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x48000000U



/*
 * Base addresses of peripherals hanging on APB bus
 */


#define USART1_BASEADDR 						(APBPERIPH_BASEADDR + 0x13800)
#define USART2_BASEADDR 						(APBPERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 						(APBPERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR							(APBPERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR 						(APBPERIPH_BASEADDR + 0x5000)
#define USART6_BASEADR 							(APBPERIPH_BASEADDR + 0x11400)
#define USART7_BASEADDR 						(APBPERIPH_BASEADDR + 0x11800)
#define USART8_BASEADDR 						(APBPERIPH_BASEADDR + 0x11C00)
#define I2C1_BASEADDR							(APBPERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR							(APBPERIPH_BASEADDR + 0x5800)
#define SPI1_I2S1_BASEADDR						(APBPERIPH_BASEADDR + 0x13000)
#define SPI2_BASEADDR							(APBPERIPH_BASEADDR + 0x3800)
#define SYSCFG_BASEADDR							(APBPERIPH_BASEADDR + 0x10000)
#define EXTI_BASEADDR							(APBPERIPH_BASEADDR + 0x10400)



/*
 * Base addresses of peripherals hanging on AHB1 bus
 */


#define RCC_BASEADDR							(AHB1PERIPH_BASEADDR + 0x1000)




/*
 * Base addresses of peripherals hanging on AHB2 bus
 */

#define GPIOA_BASEADDR								(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR								(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR								(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR								(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR								(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR								(AHB2PERIPH_BASEADDR + 0x1400)






/****************peripheral register definition structures *****************/


typedef struct
{
	volatile uint32_t MODER;				/*GPIO port mode register*/					/*address offset 0x00*/
	volatile uint32_t OTYPER;				/*GPIO port output type register*/			/*address offset 0x04*/
	volatile uint32_t OSPEEDR;				/*GPIO port output speed register*/			/*address offset 0x08*/
	volatile uint32_t PUPDR;				/*GPIO port pull-up/pull-down register*/	/*address offset 0x0C*/
	volatile uint32_t IDR;					/*GPIO port input data register*/			/*address offset 0x10*/
	volatile uint32_t ODR;					/*GPIO port output data register*/			/*address offset 0x14*/
	volatile uint32_t BSRR;					/*GPIO port bit set/reset register*/		/*address offset 0x18*/
	volatile uint32_t LCKR;					/*GPIO port configuration lock register*/	/*address offset 0x1C*/
	volatile uint32_t AFR[2];				/*GPIO alternate function low register*/	/*address offset 0x20*/
	volatile uint32_t BRR;		 			/*GPIO port bit reset register*/			/*address offset 0x28*/
}GPIO_RegDef_t;



typedef struct
{
	volatile uint32_t CR;					/*Clock control register*/					/*address offset 0x00*/
	volatile uint32_t CFGR;					/*Clock configuration register*/			/*address offset 0x04*/
	volatile uint32_t CIR;					/*Clock interrupt register*/				/*address offset 0x08*/
	volatile uint32_t APB2RSTR;				/*APB peripheral reset register 2*/			/*address offset 0x0C*/
	volatile uint32_t APB1RSTR;				/*APB peripheral reset register 1*/			/*address offset 0x10*/
	volatile uint32_t AHBENR;				/*AHB peripheral clock enable register*/	/*address offset 0x14*/
	volatile uint32_t APB2ENR;				/*APB peripheral clock enable register 2*/	/*address offset 0x18*/
	volatile uint32_t APB1ENR;				/*APB peripheral clock enable register 1*/	/*address offset 0x1C*/
	volatile uint32_t BDCR;					/*RTC domain control register*/				/*address offset 0x20*/
	volatile uint32_t CSR;					/*Control/status register*/					/*address offset 0x24*/
	volatile uint32_t AHBRSTR;				/*AHB peripheral reset register*/			/*address offset 0x28*/
	volatile uint32_t CFGR2;				/*Clock configuration register 2*/			/*address offset 0x2C*/
	volatile uint32_t CFGR3;				/*Clock configuration register 3*/			/*address offset 0x30*/
	volatile uint32_t CR2;					/*Clock control register 2*/				/*address offset 0x34*/
}RCC_RegDef_t;																			/*		register definition structure		*/



typedef struct
{
	volatile uint32_t IMR;					/*Interrupt mask register*/					/*address offset 0x00*/
	volatile uint32_t EMR;					/*Event mask register*/						/*address offset 0x04*/
	volatile uint32_t RTSR;					/*Rising trigger selection register*/		/*address offset 0x08*/
	volatile uint32_t FTSR;					/*Falling trigger selection register*/		/*address offset 0x0C*/
	volatile uint32_t SWIER;				/*Software interrupt event register*/		/*address offset 0x10*/
	volatile uint32_t PR;					/*EXTI pending register*/					/*address offset 0x14*/
}EXTI_RegDef_t;


typedef struct
{
	volatile uint32_t CFGR1;
			 uint32_t RESERVED1;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR2;
}SYSCFG_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCDGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;



/*
 * Peripheral definitions ( peripheral base addresses type casted to xxx_RegDef_t)
 */


#define GPIOA 			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*) GPIOF_BASEADDR)

#define RCC				((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_I2S1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() (RCC->AHBENR |= (1 << 17) )
#define GPIOB_PCLK_EN() (RCC->AHBENR |= (1 << 18) )
#define GPIOC_PCLK_EN() (RCC->AHBENR |= (1 << 19) )
#define GPIOD_PCLK_EN() (RCC->AHBENR |= (1 << 20) )
#define GPIOE_PCLK_EN() (RCC->AHBENR |= (1 << 21) )
#define GPIOF_PCLK_EN() (RCC->AHBENR |= (1 << 22) )


/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22) )


/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14) )		//set SPI2EN to high


/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14) )
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18) )
#define USART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19) )
#define USART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5) )
#define USART7_PCLK_EN() (RCC->APB2ENR |= (1 << 6) )
#define USART8_PCLK_EN() (RCC->APB2ENR |= (1 << 7) )


/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() (RCC->APB1ENR |= (1 << 0))





/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 17) ); (RCC->AHBRSTR &= ~(1 << 17));} while(0)	//this function will be executed only once
#define GPIOB_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 18) ); (RCC->AHBRSTR &= ~(1 << 18));} while(0)	//it sets 1 do RST register and then it shifts it back to 0
#define GPIOC_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 19) ); (RCC->AHBRSTR &= ~(1 << 19));} while(0)		//this way RST register won't hang, therefore blocking
#define GPIOD_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 20) ); (RCC->AHBRSTR &= ~(1 << 20));} while(0)		//any operations on GPIOx port
#define GPIOE_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 21) ); (RCC->AHBRSTR &= ~(1 << 21));} while(0)
#define GPIOF_REG_RESET() 		do{(RCC->AHBRSTR |= (1 << 22) ); (RCC->AHBRSTR &= ~(1 << 22));} while(0)


/*
 * GPIO port code for given GPIOx address
 */

#define GPIO_BASEADDR_TO_CODE(x)   ((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:0)
/*
 * Clock disable macros for I2Cx peripherals
 */


/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }
#define SPI2_PCLK_DI() {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }

/*
 * Clock disable macros for USARTx peripherals
 */



/*
 * Clock disable macros for SYSCFG peripherals
 */


/*
 * Clock disable macros for GPIOx peripherals
 */

/*
 * IRQ numbers of STM32f07xx MCU
 */

#define IRQ_NO_EXTI0_1		5
#define IRQ_NO_EXTI2_3		6
#define IRQ_NO_EXTI4_15		7



/*
 * Some generic macros
 */

/*
 ******************** Macros for SPI Peripheral***************************
 */

/*
 * Control Register 1 macros
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3	//BAUD RATE bit
#define SPI_CR1_SPE			6	//SPI Enable bit
#define SPI_CR1_LSBFIRST	7	//If set to 1, LSB is transmitted/received first
#define SPI_CR1_SSI			8	//Internal Slave select
#define SPI_CR1_SSM			9	//Software management
#define SPI_CR1_RXONLY		10	//Receive only mode enabled
#define SPI_CR1_CRCL		11	//CRC Length
#define SPI_CR1_CRCNEXT		12	//Transmit CRC next from Tx CRC register
#define SPI_CR1_CRCEN		13	//Hardware CRC calculation enable
#define SPI_CR1_BIDIOE		14	//Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE	15	//Bidirectional data mode enable

/*
 * Control Register 2 macros
 */
#define SPI_CR2_RXDMAEN		0	//Rx buffer DMA enable
#define SPI_CR2_TXDMAEN		1	//Tx buffer DMA enable
#define SPI_CR2_SSOE		2	//SS output enable
#define SPI_CR2_NSSP		3	//NSS pulse management
#define SPI_CR2_FRF			4	//Frame format
#define SPI_CR2_ERRIE		5	//Error interrupt enable
#define SPI_CR2_RXEIE		6	//RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE		7	//Tx buffer empty interrupt enable
#define SPI_CR2_DS			8	//Data size - USES 4 BITFIELDS
#define SPI_CR2_FRXTH		12	//FIFO reception threshold
#define SPI_CR2_LDMA_RX		13	//Last DMA transfer for reception
#define SPI_CR2_LDMA_TX		14	//Last DMA transfer for transmission


/*
 * Status Register macros
 */

#define SPI_SR_RXNE			0	//Receive buffer not empty
#define SPI_SR_TXE			1	//Transmit buffer empty
#define SPI_SR_CHSIDE		2	//Channel side
#define SPI_SR_UDR			3	//Underrun flag
#define SPI_SR_CRCERR		4	//CRC error flag
#define SPI_SR_MODF			5	//Mode fault
#define SPI_SR_OVR			6	//Overrun flag
#define SPI_SR_BSY			7	//Busy flag
#define SPI_SR_FRE			8	//Frame format error
#define SPI_SR_FRLVL		9	//FIFO reception level - USES 2 BITFIELDS
#define SPI_SR_FTLVL		11	//FIFO Transmission Level - USES 2 BITFIELDS





#define ENABLE 							1
#define DISABLE 						0
#define SET 							ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define FLAG_SET						SET
#define FLAG_RESET						RESET


#include "stm32f072Tx_gpio_driver.h"
#include "stm32f072Tx_spi_driver.h"

#endif /* INC_STM32F072XX_H_ */
