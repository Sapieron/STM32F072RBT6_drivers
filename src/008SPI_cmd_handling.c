/*
 * 008SPI_cmd_handling.c
 *
 *  Created on: 09.07.2019
 *      Author: Pawel
 */



#include "stm32f072xx.h"
#include "string.h"
#include "stm32f072Tx_spi_driver.h"
#include "stm32f072Tx_gpio_driver.h"
#include <stdio.h>


/*
 * PA15 -> SPI1 NSS
 * PB3  -> SPI1 CSK
 * PB4  -> SPI1-MISO
 * PB5  -> SPI1 MOSI
 *
 * AF0
 */

/*
 * Command Codes for arduino slave
 */

#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT         	  0x53
#define COMMAND_ID_READ       	  0x54


#define LED_PIN					9
#define LED_ON  		  		 1
#define LED_OFF 				   0

//arduino analog pins
#define ANALOG_PIN0			   0
#define ANALOG_PIN1	   			1
#define ANALOG_PIN2  			 2
#define ANALOG_PIN3  			 3
#define ANALOG_PIN4   			4




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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO4;
	GPIO_Init(&SPIPins);

	//MOSI pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_Init(&SPIPins);
}

void SPI1_Init()
{
	SPI_Handle_t SPIhandle;

	SPIhandle.pSPIx = SPI1;
	SPIhandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPIhandle.SPIConfig.SPI_DSIZE = SPI_DSIZE_8BITS;
	SPIhandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIhandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPIhandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIhandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIhandle.SPIConfig.SPI_SSM = SPI_SSM_EN;				//Software slave management


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


int main(void)
{
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;
	uint8_t ackByte = 0x00;
	uint8_t args[2];
	uint8_t dataRead = 0;

	SPI1_GPIO_Init();

	GPIO_ButtonInit();

	SPI1_Init();

	SPI_SSOEControl(SPI1, ENABLE);

	SPI_PeriphControl(SPI1, ENABLE);

	while(1)
	{

		uint8_t command = COMMAND_LED_CTRL;

		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO0));

		delay();

		SPI_PeriphControl(SPI1, ENABLE);




		//Send command to slave
		SPI_SendData(SPI1, &command, 1);

		SPI_ReceiveData(SPI1, &dummyRead, 1);	//it must be done to clear the RXNE flag


		//fetch acknowledged byte by sending dummy byte
		SPI_SendData(SPI1, &dummyWrite, 1);
		SPI_ReceiveData(SPI1, &ackByte, 1);



		//check if the byte is ack or nack
		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI1, args, 2);
		}
		//SPI_ReceiveData(SPI1, &dummyRead, 2);
		//while(SPI_GetFlag(SPI1, SPI_BUSY_FLAG));

		//SPI_PeriphControl(SPI1, DISABLE);

		/*
		 * Sensor read
		 */
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO0));
		delay();



		command = COMMAND_SENSOR_READ;


		//Send command to slave
		SPI_SendData(SPI1, &command, 1);

		SPI_ReceiveData(SPI1, &dummyRead, 1);	//it must be done to clear the RXNE flag


		//fetch acknowledged byte by sending dummy byte
		SPI_SendData(SPI1, &dummyWrite, 1);		//TODO tego ju¿ nie dostajê od slave'a - z debugera wynika, ¿e mi wisi flaga
		SPI_ReceiveData(SPI1, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;

			SPI_SendData(SPI1, args, 1);
			SPI_ReceiveData(SPI1, &dummyRead, 1);

			//ADC conversion may take a second, that's why small delay is needed

			delay();

			SPI_SendData(SPI1, &dummyWrite, 1);
			SPI_ReceiveData(SPI1, &dataRead, 1);
		}


		//while(SPI_GetFlag(SPI1, SPI_BUSY_FLAG));

		//SPI_PeriphControl(SPI1, DISABLE);
	}

	return 0;
}
