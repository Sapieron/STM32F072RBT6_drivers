/*
 * stm32f072Tx_spi_driver.c
 *
 *  Created on: 28.05.2019
 *      Author: Pawel
 */


#include "stm32f072Tx_spi_driver.h"
#include "stm32f072xx.h"


/*
 * helper functions
 */


void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR2 & (SPI_DSIZE_8BITS << SPI_CR2_DS))
	{
		*(uint8_t*)&pSPIHandle->pSPIx->DR |= *(uint8_t*)pSPIHandle->pTxBuffer;
	}
}



void spi_rxne_interrupt_handle()
{
	//
}



void spi_ovr_err_interrupt_handle()
{

}



/*
 * @fn 			- SPI_PeriClockControl
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}
				if(pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}
	}
}


/*
 * @fn 			- SPI_Init
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_Init(SPI_Handle_t *pHandle)
{
	uint32_t temp = 0, temp2 = 0;

	SPI_PeriClockControl(pHandle->pSPIx, ENABLE);

	pHandle->pSPIx->CR1 = temp;
	pHandle->pSPIx->CR2 = temp2;

	temp |= pHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	temp |= pHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	temp |= pHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	temp2 |= pHandle->SPIConfig.SPI_DSIZE << SPI_CR2_DS;

//	temp |= ENABLE << SPI_CR1_SPE;										//Set bit SPE (SPI ENABLE) to enable - set it later, as it causes MODF error
	temp |= pHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULLDUPLEX)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);			//set BIDIrectional mode to 0 - 2 line unidirectional is enabled
	} else if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALFDUPLEX)
	{
		temp |= (1 << SPI_CR1_BIDIMODE);
	} else if(pHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);							//Set RX ONLY MODE bit
	}

	temp |= pHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	pHandle->pSPIx->CR1 = temp;
	pHandle->pSPIx->CR2 = temp2;
	pHandle->pSPIx->CR2 |= (1 << 3);			//SET NSSP (NSS PULSE MANAGEMENT) - nss pulse is generated adter the transfer
}

/*
 * @fn 			- SPI_DeInit
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_DeInit(SPI_Handle_t *pSPIx)
{
	//TODO
}


/*
 * @fn 			- SPI_GetFlag
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

uint8_t SPI_GetFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}

/*
 * @fn 			- SPI_SendData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-	This is blocking call
 *
 */





void SPI_SendData(SPI_RegDef_t *pSPIx, volatile uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len!=0)
	{
		//1. wait until TXE buffer is empty
		while(SPI_GetFlag(pSPIx, SPI_TXE_FLAG ) == FLAG_RESET );	//here we are polling for the TXE flag to set, which blocks the function for time the flag sets
		//2. check DS register

		/*if(pSPIx->CR2 & (SPI_DSIZE_4BITS << SPI_CR2_DS ))
		{
			//4 Bits
			pSPIx->DR |= *pTxBuffer;
			pTxBuffer++;
			--Len;

		} else if(pSPIx->CR2 & (SPI_DSIZE_5BITS << SPI_CR2_DS ))
		{
			//5 Bits
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_6BITS << SPI_CR2_DS ))
		{
			//6 Bits
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_7BITS << SPI_CR2_DS ))
		{
			//7 Bits
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			--Len;
		} else */ if(pSPIx->CR2 & (SPI_DSIZE_8BITS << SPI_CR2_DS ))
		{
			//8 Bits

			*(volatile uint8_t *)&pSPIx->DR = *pTxBuffer;		//type casting DR to uint8_t was required, as DR register
																//is 2 bytes long, resulting in sending one byte with ,,0"
			--Len;
			pTxBuffer++;

		} else if(pSPIx->CR2 & (SPI_DSIZE_9BITS << SPI_CR2_DS ))
		{
			//9 Bits													//from now all of these are more than 1 byte
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_10BITS << SPI_CR2_DS ))
		{
			//10 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_11BITS << SPI_CR2_DS ))
		{
			//11 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_12BITS << SPI_CR2_DS ))
		{
			//12 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_13BITS << SPI_CR2_DS ))
		{
			//13 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_14BITS << SPI_CR2_DS ))
		{
			//14 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_15BITS << SPI_CR2_DS ))
		{
			//15 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		} else if(pSPIx->CR2 & (SPI_DSIZE_16BITS << SPI_CR2_DS ))
		{
			//16 Bits
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer++;
			--Len;
			--Len;
		}
	}
}


/*
 * @fn 			- SPI_ReceiveData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, volatile uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len!=0)
	{
		while(SPI_GetFlag(pSPIx, SPI_RXNE_FLAG) == FLAG_SET);

		if(pSPIx->CR2 & (SPI_DSIZE_8BITS << SPI_CR2_DS))
		{
			*pRxBuffer = *(volatile uint8_t *)&pSPIx->DR;
			//pRxBuffer++;
			--Len;
		}
	}
}

/*
 * SPI Send Data - interrupt based
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		//Save TX buffer and Len to global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//Mark SPI state as busy so no other code can take over SPI control until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Set TXEIE to high, so that TXE flag generates interrupt request
		pSPIHandle->pSPIx->CR2 |= (SET << SPI_CR2_TXEIE);

	}


	return state;
}


/*
 * SPI Receive data - interrupt based
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t status = pSPIHandle->TxState;
	if (status != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState |= SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->CR2 |= (SET << SPI_CR2_RXEIE);
	}
	return status;
}

/*
 *@fn 			- SPI_VerifyResponse
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note
 *
 */




uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5)
		return 1;
	else
		return 0;
}




/*
 * @fn 			- SPI_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


/*
 * @fn 			- SPI_IRQPriorityConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}


/*
 * @fn 			- SPI_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	//check for TXE and TXEIE
	temp1 = pHandle->pSPIx->SR & (SET << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (SET << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//TODO
	}
	//check for RXNE and RXNEIE
	temp1 = pHandle->pSPIx->SR & (SET << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (SET << SPI_CR2_RXEIE);

	if(temp1 && temp2)
	{
		//TODO
	}

	//check for OVR (overrun flag)
	temp1 = pHandle->pSPIx->SR & (SET << SPI_SR_OVR);			//check if overrun flag is set
	temp2 = pHandle->pSPIx->CR2 & (SET << SPI_CR2_ERRIE);		//check if error detection reg is set

	if(temp1 && temp2)
	{
		//TODO
	}
}

/*
 * @fn 			- SPI_PeriphControl
 *
 * @brief		- This functions sets SPE bit of CR1 register
 * 				  enabling the peripheraL
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */



void SPI_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * @fn 			- SPI_SSIControl
 *
 * @brief		- SSI must be set to 1 in order the NSS pin
 * 				  is pulled to vcc. That way MSTR error flag
 * 				  does not occur.
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */


void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}


/*
 * @fn 			- SPI_SOEControl
 *
 * @brief		-
 *
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */


void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}
