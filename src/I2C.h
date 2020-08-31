#ifndef _I2C_H
#define _I2C_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define STATUS_REG_BIT_MASK 0xf8

#define I2C_STATUS_BUS_0 (TWSR0 & STATUS_REG_BIT_MASK)
#define I2C_STATUS_BUS_1 (TWSR1 & STATUS_REG_BIT_MASK)

#define START_SUCCEEDED 			0x08
#define REPEAT_START_SUCCEEDED 		0x10
#define SLAVE_WRITE_SUCCEEDED 		0x18
#define SLAVE_READ_SUCCEEDED 		0x40
#define SLAVE_SENT_NEXT_BYTE		0x50
#define DATA_TRANSMIT_SUCCEEDED		0x28
#define END_READ_SUCCEEDED			0x58

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/


/******************************************************************* Notice ************************************************************************
* This module is highly custom to this project. To be optimized for speed, it is built under the assumption that there is a single slave on each bus
* and that the microcontroller is master of both buses. Changes will need to be made if either of those assumptions stop holding true.
*/

/**
* Initializes internal parameters.
* Should be called exactly once before anything is attempted to be done with the particular bus.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_Init(uint8_t bus);

/**
* Begins the I2C exchange.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_Start(uint8_t bus);

/**
* Repeats start.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_RepeatStart(uint8_t bus);

/**
* Begins a write communication with the slave with the given address.
* Non blocking.
* @param[in]		slaveAddress 		the address of the slave the master wishes to write to.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_SendSlaveAddressWrite(uint8_t slaveAddress, uint8_t bus);

/**
* Begins a read communication with the slave with the given address.
* @param[in]		slaveAddress 		the address of the slave the master wishes to read from.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_SendSlaveAddressRead(uint8_t slaveAddress, uint8_t bus);

/**
* Transmits the address of the register the user wishes to R/W to
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* Non blocking.
* @param[in]		registerAddress 		the address of the register the master wishes to write to.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_SendSlaveRegister(uint8_t registerAddress, uint8_t bus);

/**
* Transmits the data the user wishes to send to the appropriate slave and register.
* Non blocking.
* @param[in]		data 			the byte of data the master wishes to write.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_SendData(uint8_t data, uint8_t bus);

/**
* Asks the slave to send more data.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_AskForAnotherByte(uint8_t bus);

/**
* returns the data sent over by the slave.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
* @return the byte just sent over.
*/
uint8_t I2C_Read(uint8_t bus);

/**
* Alerts the slave that no more data is required.
* Should be called to end a multi byte read, right before the stop function is called.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_EndDataRead(uint8_t bus);

/**
* Ends communications and frees the I2C Bus.
* Non blocking.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_Stop(uint8_t bus);

/**
* Disables the I2C interrupt on the specified bus.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_DisableInterrupt(uint8_t bus);

/**
* Blocks until the specified bus is ready for the next operation.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_BlockUntilReady(uint8_t bus);

/**
* Enables the I2C interrupt on the specified bus.
* @param[in]		bus 		the bus of interest. Options are 0 and 1. Undefined behavior will occur for any other value.
*/
void I2C_EnableInterrupt(uint8_t bus);

#endif // _I2C_H
