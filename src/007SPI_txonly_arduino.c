/*
 * 007SPI_txonly_arduino.c
 *
 *  Created on: 03.06.2019
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
	GPIO_Handle_t SPIPins, SPIPins2;



	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;							//Set AF0, which lets SPI to work
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_HI;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	SPIPins2.pGPIOx = GPIOA;

	SPIPins2.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	SPIPins2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;							//Set AF0, which lets SPI to work
	SPIPins2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUTP_SPEED_HI;
	SPIPins2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTP_TYPE_PP;
	SPIPins2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//NSS pin config
	SPIPins2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO15;
	GPIO_Init(&SPIPins2);

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

	SPIhandle.pSPIx = SPI1;
	SPIhandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPIhandle.SPIConfig.SPI_DSIZE = 0;								// TODO wczeœniej by³o SPI_DSIZE_8BITS
	SPIhandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIhandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPIhandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIhandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIhandle.SPIConfig.SPI_SSM = SPI_SSM_EN;				//Hardware slave management


	SPI_Init(&SPIhandle);

	//This makes internal SSI signal as high, thus avoiding MODF error
	//SPI_SSIControl(SPI1, ENABLE);		//not required in hardware slave management

}


void GPIO_ButtonInit()
{
	GPIO_Handle_t gpioButton;
	memset(&gpioButton,0,sizeof(gpioButton));
	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPT;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO0;

	GPIO_Init(&gpioButton);
}

void delay()
{
	for(uint32_t i = 0; i<250000; ++i);
}


char data_string[] = "Chwalmy papieza polaka";


int main(void)
{
	SPI1_GPIO_Init();

	GPIO_ButtonInit();

	SPI1_Init();

	SPI_SSOEControl(SPI1, ENABLE);
	while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO0));

		delay();

		SPI_PeriphControl(SPI1, ENABLE);

		//Send the length of data to slave
		uint8_t dataLength = strlen(data_string);
		SPI_SendData(SPI1, &dataLength, 1);


		SPI_SendData(SPI1, (uint8_t*)data_string, strlen(data_string));

		while(SPI_GetFlag(SPI1, SPI_BUSY_FLAG));

		SPI_PeriphControl(SPI1, DISABLE);
	}

	return 0;
}
