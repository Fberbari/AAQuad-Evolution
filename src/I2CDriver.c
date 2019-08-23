#include "I2CDriver.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define STATUS_REG_BIT_MASK 0xf8

#define STATUS (TWSR0 & STATUS_REG_BIT_MASK)

#define START_SUCCEEDED 			0x08
#define REPEAT_START_SUCCEEDED 		0x10
#define SLAVE_WRITE_SUCCEEDED 		0x18
#define SLAVE_READ_SUCCEEDED 		0x40
#define SLAVE_SENT_NEXT_BYTE		0x50
#define DATA_TRANSMIT_SUCCEEDED		0x28
#define END_READ_SUCCEEDED			0x58

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void I2CDriver_Init(void)
{
	TWSR0 &= ~( (1 << TWPS0) | (1 << TWPS1) );	// no prescaler
	TWBR0 = (uint8_t) ((((float) F_CPU / I2C_BIT_RATE) - 16.0f ) / 2.0f);
}

int I2CDriver_Start(void)
{
	TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) ); 

	while(! (TWCR0 & (1 << TWINT)) ); 

	if ( STATUS != START_SUCCEEDED) 
	{
		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_RepeatStart(void)
{

	TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) );

	while(! (TWCR0 & (1 << TWINT)) ); 

	if (STATUS != REPEAT_START_SUCCEEDED) 
	{
		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_SendSlaveAddressWrite(uint8_t slaveAddress)
{

	TWDR0 = ( slaveAddress << 1 );

	TWCR0 = ( (1 << TWINT) | (1 << TWEN) );


	while(! (TWCR0 & (1 << TWINT)) ); 

	if (STATUS != SLAVE_WRITE_SUCCEEDED) 
	{ 

		return AAQUAD_FAILED;
	}

	return AAQUAD_SUCCEEDED;

}

int I2CDriver_SendSlaveAddressRead(uint8_t slaveAddress)
{

	TWDR0 = ( slaveAddress << 1 ) + 1;

	TWCR0 = ( (1 << TWINT) | (1 << TWEN) );


	while(! (TWCR0 & (1 << TWINT)) );

	if (STATUS != SLAVE_READ_SUCCEEDED) 
	{ 

		return AAQUAD_FAILED;
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_SendSlaveRegister(uint8_t registerAddress)
{

	TWDR0 = registerAddress; 

  	TWCR0 = ( (1 << TWINT) | (1 << TWEN) );

	while(! (TWCR0 & (1 << TWINT)) );

	if ( (STATUS != DATA_TRANSMIT_SUCCEEDED) ){

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_SendData(uint8_t data)
{
	TWDR0 = data;

	TWCR0 = ((1 << TWINT) | (1 << TWEN));
	
	while(! (TWCR0 & (1 << TWINT)) ); 

	if ( (STATUS != DATA_TRANSMIT_SUCCEEDED) ){

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_WaitForNextReadByte(void)
{
	TWCR0 = ( (1 << TWEN) | (1 << TWINT) | (1 << TWEA));

	while (! (TWCR0 & (1 << TWINT)) );

	if ( STATUS != SLAVE_SENT_NEXT_BYTE)
	{

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_EndDataRead(void)
{
	TWCR0 = ( (1 << TWEN) | (1 << TWINT) );

	while (! (TWCR0 & (1 << TWINT)) );

	if (STATUS != END_READ_SUCCEEDED)
	{
		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

void I2CDriver_Stop(void)
{
	TWCR0 |= ( (1 << TWEN) | (1 << TWINT) | (1 << TWSTO) ); 
}
