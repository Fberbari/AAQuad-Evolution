#ifndef _I2CDRIVER_H
#define _I2CDRIVER_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define I2C_BIT_RATE 50000.0f	// in Hz

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void I2CDriver_Init(void);

/**
* Begins the I2C exchange
* Will return AAQuad Failed if the hardware fails or if the bus is busy.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_Start(void);

/**
* Repeats start.
* Will return AAQuad Failed if the hardware fails.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_RepeatStart(void);

/**
* Begins a write communication with the slave with the given address.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		slaveAddress 		the address of te slave the master wishes to write to.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_SendSlaveAddressWrite(uint8_t slaveAddress);

/**
* Begins a read communication with the slave with the given address.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		slaveAddress 		the address of the slave the master wishes to read from.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_SendSlaveAddressRead(uint8_t slaveAddress);

/**
* Blocks while slave transmitts the next byte in a multi-read command.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_WaitForNextReadByte(void);

/**
* Transmits the address of the register the user wishes to R/W to
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		registerAddress 		the address of te register the master wishes to write to.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_SendSlaveRegister(uint8_t registerAddress);

/**
* Transmits the data the user wishes to send to the appropriate slave and register.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		data 			the byte of data the master wishes to write.
* @return 			AAQUAD_FAILED or AAQUAD_SUCCEEDED
*/
int I2CDriver_SendData(uint8_t data);

/**
* Alerts the slave that no more data is required.
* Should be called to end a multi byte read, right before the stop function is called.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
*/
int I2CDriver_EndDataRead(void);

/**
* Ends communications and frees the I2C Bus.
*/
void I2CDriver_Stop(void);

#endif // _I2CDRIVER_H
