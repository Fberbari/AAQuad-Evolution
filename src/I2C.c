#include "I2C.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static bool InterruptEnabled = false;

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void I2C_Init(void)
{
	// Corresponds to a 100 KHz SCL when F_CPU is 20 MHz.
	TWSR0 &= ~( (1 << TWPS0) | (1 << TWPS1) );	// no prescaler
	TWBR0 = 0x5C;
}

void I2C_Start(void)
{
	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) );
	}
}

void I2C_RepeatStart(void)
{
	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) );
	}
}

void I2C_SendSlaveAddressWrite(uint8_t slaveAddress)
{
	TWDR0 = ( slaveAddress << 1 );

	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) );
	}
}

void I2C_SendSlaveAddressRead(uint8_t slaveAddress)
{
	TWDR0 = ( slaveAddress << 1 ) + 1;
	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) );
	}
}

void I2C_SendSlaveRegister(uint8_t registerAddress)
{
	TWDR0 = registerAddress;
	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) );
	}
}

void I2C_SendData(uint8_t data)
{
	TWDR0 = data;
	if (InterruptEnabled)
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
	}
	else
	{
		TWCR0 = ( (1 << TWINT) | (1 << TWEN) );
	}
}

void I2C_Stop(void)
{
	TWCR0 = ( (1 << TWEN) | (1 << TWINT) | (1 << TWSTO) );
}

void I2C_DisableInterrupt(void)
{
	InterruptEnabled = false;
}

void I2C_BlockUntilReady(void)
{
	while( !(TWCR0 & (1 << TWINT)) );
}

void I2C_EnableInterrupt(void)
{
	InterruptEnabled = true;
}
