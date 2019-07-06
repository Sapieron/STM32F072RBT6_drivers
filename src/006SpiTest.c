/*
 * 006SpiTest.c
 *
 *  Created on: 02.06.2019
 *      Author: Pawel
 */


#include "stm32f072xx.h"
#include "string.h"
#include "stm32f072Tx_spi_driver.h"
#include "stm32f072Tx_gpio_driver.h"

/*
 * PA15 -> SPI1 NSS
 * PB3  -> SPI1 CSK
 * PB4  -> SPI1-MISO
 * PB5  -> SPI1 MOSI
 *
 * AF0
 */




void SPI1_GPIO_Init()
{
	GPIO_Handle_t SPIPins;

	//memset(&SPIPins, 0, sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_HI;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	//NSS pin config
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO;
	//GPIO_Init(&SPIPins);

	//SCK pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO3;
	GPIO_Init(&SPIPins);

	//MISO pin config
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO;
	//GPIO_Init(&SPIPins);

	//MOSI pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_Init(&SPIPins);
}

void SPI1_Init()
{
	SPI_Handle_t SPIhandle;

	//memset(&SPIhandle, 0, sizeof(SPIhandle));

	SPIhandle.pSPIx = SPI1;
	SPIhandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPIhandle.SPIConfig.SPI_DSIZE = SPI_DSIZE_8BITS;
	SPIhandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIhandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPIhandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIhandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIhandle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPIhandle);

	//TODO - uwaga, tu doda³em ustawienie CRCL - wg dokumenctacji, a wg debuggera to jest DFF!!!
	//SPI1->CR1 |= (1 << SPI_CR1_CRCL);

	//This makes internal SSI signal as high, thus avoiding MODF error
	SPI_SSIControl(SPI1, ENABLE);

}

char data_string[] = "Hello world";


int main(void)
{
	SPI1_GPIO_Init();


	SPI1_Init();

	SPI_PeriphControl(SPI1, ENABLE);

	SPI_SendData(SPI1, (uint16_t*)data_string, strlen(data_string));

	while(SPI_GetFlag(SPI1, SPI_BUSY_FLAG));

	SPI_PeriphControl(SPI1, DISABLE);
	while(1);

	return 0;
}
