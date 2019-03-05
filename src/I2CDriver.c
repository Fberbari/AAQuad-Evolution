#include "I2CDriver.h"
#include <avr/io.h>

void I2CDriver_Init(void)
{
	TWBR0 = (uint8_t) ( F_CPU / ( BIT_RATE * 1000) );
}

int I2CDriver_Start(void)
{
	TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) ); // writes the start condition on the line  and Hardware will clear this bit when ready

	while(! (TWCR0 & (1 << TWINT)) ); // Hardware will write this to 0 when ready to go

	if ( (TWSR0 & 0xf8) != 0x08){ // comfirms that status is infact start condition has gone through

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_RepeatStart(void)
{

	TWCR0 = ( (1 << TWEN) | (1 << TWSTA ) | (1 << TWINT) ); // writes the start condition on the line  and Hardware will clear this bit when ready


	while(! (TWCR0 & (1 << TWINT)) ); // Hardware will write this to 0 when ready to go

	if ( (TWSR0 & 0xf8) != 0x10){ // comfirms reapeated start

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_SendSlaveAddressWrite(uint8_t slaveAddress)
{
	// send slave address + write bit

	TWDR0 = ( address << 1 );

	TWCR0 = ( (1 << TWINT) | (1 << TWEN) );


	while(! (TWCR0 & (1 << TWINT)) ); // Hardware will write this to 0 when ready to go

	if ( ( (TWSR0 & 0xf8) != 0x18) && ( (TWSR0 & 0xf8) != 0x40) ) { // confirms that slave has received address and sent ACK

		return AAQUAD_FAILED;
	}

	return AAQUAD_SUCCEEDED;

}

int I2CDriver_SendSlaveRegister(uint8_t registerAddress)
{
	// send  address of register to be written

	TWDR0 = reg; 

  	TWCR0 = ( (1 << TWINT) | (1 << TWEN) );

	while(! (TWCR0 & (1 << TWINT)) ); // Hardware will write this to 0 when ready to go

	if ( ((TWSR0 & 0xf8) != 0x28) ){ // confirms that slave has received address of register and sent ACK

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

int I2CDriver_SendData(uint8_t data)
{
	TWDR0 = data;

	TWCR0 = ((1 << TWINT) | (1 << TWEN));
	
	while(! (TWCR0 & (1 << TWINT)) ); // Hardware will write this to 0 when ready to go

	if ( ((TWSR0 & 0xf8) != 0x28) ){ // comfirms that slave has accepted data and sent ACK

		return AAQUAD_FAILED; 
	}

	return AAQUAD_SUCCEEDED;
}

void I2CDriver_Stop(void)
{
	TWCR0 |= ( (1 << TWEN) | (1 << TWINT) | (1 << TWSTO) ); 
}
