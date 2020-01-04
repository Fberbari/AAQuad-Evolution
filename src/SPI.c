#include "SPI.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define SCK_PIN 1
#define MISO_PIN 0
#define MOSI_PIN 3
#define SS_PIN 2

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void SPI_Init(void)
{
	DDRC |= (1 << SCK_PIN);
	DDRC &= ~(1 << MISO_PIN);
	DDRE |= (1 << MOSI_PIN);
	DDRE |= (1 << SS_PIN);

	SPI_EndTransaction();

	SPCR1 = ( (1 << SPIE1)|				// SPI Interupt Enable
			  (1 << SPE1) |				// SPI Enable
			  (0 << DORD1)|				// MSB first
			  (1 << MSTR1)|				// Master mode
			  (0 << CPOL1)|				// CPOL 0
			  (0 << CPHA1)|				// CPHA 0
			  (0 << SPR11)|(0 << SPR10)	// Clk div by 4
		);

    SPSR1 = (1 << SPI2X1); // Double clk speed
}

void SPI_Write(uint8_t byte)
{
	SPDR1 = byte;
}

uint8_t SPI_Read(void)
{
	return SPDR1;
}

uint8_t SPI_ReadWriteBlocking(uint8_t byte)
{
	SPCR1 &= ~(1 << SPIE1);
	SPDR1 = byte;
	while (!(SPSR1 & (1 << SPIF1)))
	{
		asm("nop");
	}
	uint8_t receivedData = SPDR1;
	SPCR1 |= (1 << SPIE1);
	return receivedData;
}

void SPI_BeginTransaction(void)
{
	PORTE &= ~(1 << SS_PIN);
}

void SPI_EndTransaction(void)
{
	PORTE |= (1 << SS_PIN);
}