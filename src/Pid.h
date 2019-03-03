#ifndef _PID_H
#define _PID_H

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
void Pid_Init(void);

/**
* Combines the pilot instructions and the sensor data to compute the required power each motor must have.
* Returns succeeded if it possible to achieve the pilot's intentions and failed otherwise.
* @param[in]	PilotResult 	A pointer to the struct containing the pilot's intentions.
* @param[out]	motors			An array of 4 floats that each represent a percentage from 0-100 that the associated motor should be powered at.
* @return 		AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int Pid_Compute(PilotResult_t *PilotResult, float *motors);

#endif // _PID_H
