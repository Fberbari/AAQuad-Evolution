#ifndef _SPI_H
#define _SPI_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters.
* Should be called once before anything is attempted to be done with the module.
*/
void SPI_Init(void);

/**
* Asserts the slave select.
*/
void SPI_BeginTransaction(void);

/**
* Deasserts the slave select.
*/
void SPI_EndTransaction(void);

/**
* Sends a byte on the SPI interface.
* This function returns right away, an interrupt is raised when
* the send completes.
* @param[in]		byte	the byte to send on the interface.
*/
void SPI_Write(uint8_t byte);

/**
* Returns the byte received from the spi interface.
* This function is non blocking and returns right away.
* This function is meant to be called from an ISR right after the send is complete.
* @return		byte	the byte received.
*/
uint8_t SPI_Read(void);

/**
* Sends a byte on the SPI interface, waits for send complete and returns the incoming byte.
* This function disables the spi interrupt for its duration and reenables it after it returns.
* @param[in]		byte	the byte to send on the interface.
* @return 			the byte received from the interface.
*/
uint8_t SPI_ReadWriteBlocking(uint8_t byte);

#endif // _SPI_H