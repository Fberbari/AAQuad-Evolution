#include "I2C.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static bool InterruptEnabled[2];

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void I2C_Init(uint8_t bus)
{
    uint8_t statusRegMask = ( (1 << TWPS0) | (1 << TWPS1) );
    uint8_t bitRateReg = 0x5C; // Corresponds to a 100 KHz SCL when F_CPU is 20 MHz.

    if (bus == 0)
    {
        TWSR0 &= ~statusRegMask;
        TWBR0 = bitRateReg;
    }

    else if (bus == 1)
    {
        TWSR1 &= ~statusRegMask;
        TWBR1 = bitRateReg;
    }
}

void I2C_Start(uint8_t bus)
{
    uint8_t controlReg = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) );

    if( InterruptEnabled[bus] )
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWCR1 = controlReg;
    }
}

void I2C_RepeatStart(uint8_t bus)
{
    uint8_t controlReg = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) );

	if (InterruptEnabled[bus])
	{
		controlReg |= (1 << TWIE);
	}

    if (bus == 0)
    {
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWCR1 = controlReg;
    }
}

void I2C_SendSlaveAddressWrite(uint8_t slaveAddress, uint8_t bus)
{
	uint8_t dataReg = (slaveAddress << 1);
    uint8_t controlReg = ( (1 << TWINT) | (1 << TWEN) );

	if (InterruptEnabled[bus])
	{
		controlReg |= (1 << TWIE);
	}

    if (bus == 0)
    {
        TWDR0 = dataReg;
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWDR1 = dataReg;
        TWCR1 = controlReg;
    }
}

void I2C_SendSlaveAddressRead(uint8_t slaveAddress, uint8_t bus)
{
    uint8_t dataReg = (slaveAddress << 1) + 1;
    uint8_t controlReg = ( (1 << TWINT) | (1 << TWEN) );

    if (InterruptEnabled[bus])
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWDR0 = dataReg;
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWDR1 = dataReg;
        TWCR1 = controlReg;
    }
}

void I2C_SendSlaveRegister(uint8_t registerAddress, uint8_t bus)
{
    uint8_t dataReg = registerAddress;
    uint8_t controlReg = ( (1 << TWINT) | (1 << TWEN) );

    if (InterruptEnabled[bus])
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWDR0 = dataReg;
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWDR1 = dataReg;
        TWCR1 = controlReg;
    }
}

void I2C_SendData(uint8_t data, uint8_t bus)
{
    uint8_t dataReg = data;
    uint8_t controlReg = ( (1 << TWINT) | (1 << TWEN) );

    if (InterruptEnabled[bus])
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWDR0 = dataReg;
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWDR1 = dataReg;
        TWCR1 = controlReg;
    }
}

void I2C_AskForAnotherByte(uint8_t bus)
{
    uint8_t controlReg = ( (1 << TWEN) | (1 << TWINT) | (1 << TWEA));

    if (InterruptEnabled[bus])
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWCR1 = controlReg;
    }
}

uint8_t I2C_Read(uint8_t bus)
{
	uint8_t data = 0;

    if (bus == 0)
    {
        data = TWDR0;
    }

    else if (bus == 1)
    {
        data = TWDR1;
    }

	return data;
}

void I2C_EndDataRead(uint8_t bus)
{
    uint8_t controlReg = ( (1 << TWEN) | (1 << TWINT) );

    if (InterruptEnabled[bus])
    {
        controlReg |= (1 << TWIE);
    }

    if (bus == 0)
    {
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWCR1 = controlReg;
    }
}

void I2C_Stop(uint8_t bus)
{
	uint8_t controlReg = ( (1 << TWEN) | (1 << TWINT) | (1 << TWSTO) );

    if (bus == 0)
    {
        TWCR0 = controlReg;
    }

    else if (bus == 1)
    {
        TWCR1 = controlReg;
    }
}

void I2C_DisableInterrupt(uint8_t bus)
{
	InterruptEnabled[bus] = false;
}

void I2C_BlockUntilReady(uint8_t bus)
{
    if (bus == 0)
    {
        while( !(TWCR0 & (1 << TWINT)) )
        {
            asm("nop");
        }
    }

    else if (bus == 1)
    {
        while( !(TWCR1 & (1 << TWINT)) )
        {
            asm("nop");
        }
    }
}

void I2C_EnableInterrupt(uint8_t bus)
{
	InterruptEnabled[bus] = true;
}
