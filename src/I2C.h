#ifndef _I2C_H
#define _I2C_H

#include "Common.h"

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
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters.
* Should be called exactly once before anything is attempted to be done with the module.
*/
void I2C_Init(void);

/**
* Begins the I2C exchange.
* Non blocking.
*/
void I2C_Start(void);

/**
* Repeats start.
* Non blocking.
*/
void I2C_RepeatStart(void);

/**
* Begins a write communication with the slave with the given address.
* Non blocking.
* @param[in]		slaveAddress 		the address of te slave the master wishes to write to.
*/
void I2C_SendSlaveAddressWrite(uint8_t slaveAddress);

/**
* Begins a read communication with the slave with the given address.
* @param[in]		slaveAddress 		the address of the slave the master wishes to read from.
*/
void I2C_SendSlaveAddressRead(uint8_t slaveAddress);

/**
* Transmits the address of the register the user wishes to R/W to
* Will return AAQuad Failed if the hardware fails or if the slave does not respond.
* Non blocking.
* @param[in]		registerAddress 		the address of te register the master wishes to write to.
*/
void I2C_SendSlaveRegister(uint8_t registerAddress);

/**
* Transmits the data the user wishes to send to the appropriate slave and register.
* Non blocking.
* @param[in]		data 			the byte of data the master wishes to write.
*/
void I2C_SendData(uint8_t data);

/**
* Alerts the slave that no more data is required.
* Should be called to end a multi byte read, right before the stop function is called.
* Non blocking.
*/
void I2C_EndDataRead(void);

/**
* Ends communications and frees the I2C Bus.
* Non blocking.
*/
void I2C_Stop(void);

/**
* Disables the I2C interrupt.
*/
void I2C_DisableInterrupt(void);

/**
* Blocks until the bus is ready for the next operation.
*/
void I2C_BlockUntilReady(void);

/**
* Enables the I2C interrupt.
*/
void I2C_EnableInterrupt(void);

#endif // _I2C_H
