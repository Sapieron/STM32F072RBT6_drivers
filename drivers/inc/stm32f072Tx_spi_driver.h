/*
 * stm32f072Tx_spi_driver.h
 *
 *  Created on: 28.05.2019
 *      Author: Pawel
 */

#ifndef INC_STM32F072TX_SPI_DRIVER_H_
#define INC_STM32F072TX_SPI_DRIVER_H_

#include "stm32f072xx.h"

/*
 * SPI Config structur
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;		//Baud rate
	uint8_t SPI_DSIZE;			//DATA SIZE
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * SPI Handle structure
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*
 *************************SPI generic functions************************
 */




/*
 * SPI clock configuration
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/*
 * SPI initialization and deinitialization
 */

void SPI_Init(SPI_Handle_t *pSPIx);


void SPI_DeInit(SPI_Handle_t *pSPIx);

/*
 * SPI send and receive data
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, volatile uint8_t *pTxBuffer, uint32_t Len);


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName);



/*
 * SPI interrupts
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other SPI functions
 */

void SPI_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * ********************SPI Generic macros******************
 */


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0


/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FULLDUPLEX			1
#define SPI_BUS_CONFIG_HALFDUPLEX			2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	//TX is not needed, as SIMPLEX TX ONLY is nothing but full duplex communication
												//without MISO line connected


/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/*
 * @SPI_DSIZE
 */

#define SPI_DSIZE_4BITS						3
#define SPI_DSIZE_5BITS						4
#define SPI_DSIZE_6BITS						5
#define SPI_DSIZE_7BITS						6
#define SPI_DSIZE_8BITS						7
#define SPI_DSIZE_9BITS						8
#define SPI_DSIZE_10BITS					9
#define SPI_DSIZE_11BITS					10
#define SPI_DSIZE_12BITS					11
#define SPI_DSIZE_13BITS					12
#define SPI_DSIZE_14BITS					13
#define SPI_DSIZE_15BITS					14
#define SPI_DSIZE_16BITS					15

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * @SPI_SSM
 */

#define SPI_SSM_EN							1
#define SPI_SSM_DI							0			//SSM software management


/*
 * SPI Flag names
 */

#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)		//masking details
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE )


#endif /* INC_STM32F072TX_SPI_DRIVER_H_ */
