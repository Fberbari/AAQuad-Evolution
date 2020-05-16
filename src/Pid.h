#ifndef _PID_H
#define _PID_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

// The angle that the quadCopter will be at when the aileron/elevator/rudder control sticks are at 100% (in rad)
#define MAX_X_THROW 0.52f	
#define MAX_Y_THROW 0.52f
#define MAX_Z_THROW	(float) M_PI

#define MAX_ALTITUDE 1	// in meters

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before anything is attempted to be done with the module
*/
void Pid_Init(void);

/**
* Uses information about the quad's position and the pilot's intentions to compute the power each motor should be set to.
* Call this function synchronously.
* Returns succeeded if it is possible to achieve the pilot's intentions and failed otherwise.
* @param[in]	PilotResult 	A pointer to the struct containing the pilot's instructions.
* @param[in]	EulerAngles 	A pointer to the struct containing the quad's current orientation.
* @param[in]	EulerRates  	A pointer to the struct containing the time derivative of the quad's current orientation.
* @param[in]	altitude		The quad's height from the ground.
* @param[out]	motors			An array of 4 floats that each represent a percentage from 0-100 that the associated motor should be powered at to achieve the pilot's intentions.
* @return 		AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int Pid_Compute(PilotResult_t *PilotResult, EulerXYZ_t *EulerAngles, EulerRates_t *EulerRates, float altitude, float *motors);

#endif // _PID_H
