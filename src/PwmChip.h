#ifndef _PWMCHIP_H
#define _PWMCHIP_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/
/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void PwmChip_Init(void);

/**
* Sends the data contained in the array of 4 floats that each represent a motor's percentage to the pwmCHip which will
* in turn send that data out to the esc's.
* This is a blocking function that will not return until communication with the pwm chip either completed or failed.
* Returns succeeded if the data was successfully communicated to the chip via I2C, returns failed otherwise.
* @param[in]		motors 			pointer to an array of 4 floats that represent motor percentages
* @return			AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int PwmChip_Send(float *motors);


#endif // _PWMCHIP_H
