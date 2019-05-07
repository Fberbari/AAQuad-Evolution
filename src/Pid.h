#ifndef _PID_H
#define _PID_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

// The angle that the quadCopter will be at when the aileron/elevator control sticks are at 100% (in degrees)
#define MAX_X_THROW 25.0f	
#define MAX_Y_THROW 25.0f
// The rotational speed that the quadCopter will be at when the rudder control stick is at 100% (in degrees/s)
#define MAX_Z_THROW 90.0f	// TODO this is not reflecting reality

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void Pid_Init(void);

/**
* Sets the initial angles that the quad is sitting at.
* Tis function must be called exactly once after Pid_Init and before Pid_Compute.
* @param[in]		initialXAngle 		The X angle the QuadCopter is at on startup.
* @param[in]		initialYangle 		The Y angle the Quadcopter is at on startup.
*/
void Pid_SetIntialAngles(float InitialXAngle, float InitialYangle);

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
