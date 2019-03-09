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
* @return 			AAQuad_Failed or AAQuad_Succeeded
*/
int I2CDriver_Start(void);

/**
* Repeats start.
* Will return AAQuad Failed if the hardware fails.
* @return 			AAQuad_Failed or AAQuad_Succeeded
*/
int I2CDriver_RepeatStart(void);

/**
* Begins a write communication with the slave with the given address.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		slaveAddress 		the address of te slave the master wishes to write to.
* @return 			AAQuad_Failed or AAQuad_Succeeded
*/
int I2CDriver_SendSlaveAddressWrite(uint8_t slaveAddress);

/**
* Transmits the address of the register the user wishes to R/W to
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		registerAddress 		the address of te register the master wishes to write to.
* @return 			AAQuad_Failed or AAQuad_Succeeded
*/
int I2CDriver_SendSlaveRegister(uint8_t registerAddress);

/**
* Transmits the data the user wishes to send to the appropriate slave and register.
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* @param[in]		data 			the byte of data the master wishes to write.
* @return 			AAQuad_Failed or AAQuad_Succeeded
*/
int I2CDriver_SendData(uint8_t data);

/**
* Ends communications and frees the I2C Bus.
*/
void I2CDriver_Stop(void);

#endif // _I2CDRIVER_H
