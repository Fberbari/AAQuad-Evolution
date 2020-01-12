#ifndef _PWMCHIP_H
#define _PWMCHIP_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/
/**
* Initialises internal parameters.
* Should be called exactly once before anything is attempted to be done with the module.
*/
void PwmChip_Init(void);

/**
* Sends the data contained in the array of 4 floats that each represent a motor's percentage to the pwm chip which will
* in turn send that data out to the esc's.
* This is a non blocking function that returns right away.
* Returns succeeded if the bus is in a working state and has begun sending the data, busy if a previous transaction is
* still in progress and failed if a previous transaction failed.
* Note that data is only sent if the return is AAQUAD_SUCCEEDED. There is no way to re-establish contact with the chip once
* a transaction has failed and one has to reattempt to send data later if the bus is busy.
* @param[in]		motors 			pointer to an array of 4 floats that represent motor percentages
* @return			AAQUAD_SUCCEEDED, AAQUAD_BUSY or AAQUAD_FAILED
*/
int PwmChip_Send(float *motors);


#endif // _PWMCHIP_H
