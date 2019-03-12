#ifndef _PID_H
#define _PID_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

// percentage that determines what percentage of maximum motor power will be increased when 100% is increased on the x,y, and z channels
// For ex: if the quad was level, and MAX_X_THROW was 10, moving the X stick to it's 50 % position would result in 2 motors reducing their power by 5% of maximum and 2 motors increasing their power by 5% of maximum.

#define MAX_X_THROW 10.0f	
#define MAX_Y_THROW 10.0f
#define MAX_Z_THROW 10.0f

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
* @param[in]	SensorResults 	A pointer to the struct containing the quad's current position.
* @param[out]	motors			An array of 4 floats that each represent a percentage from 0-100 that the associated motor should be powered at.
* @return 		AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors);

#endif // _PID_H
