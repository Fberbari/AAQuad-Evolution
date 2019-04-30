#ifndef _PID_H
#define _PID_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

// The angle that the quadCopter will be at when the aileron/elevator control sticks are at 100% (in degrees)
#define MAX_X_THROW 15.0f	
#define MAX_Y_THROW 15.0f
// The rotational speed that the quadCopter will be at when the rudder control stick is at 100% (in degrees/s)
#define MAX_Z_THROW 30.0f


 typedef struct 
 {
	float x;				
	float y;

 }InitialAngles_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void Pid_Init(void);

/**
* Meant To Be called by Calibration.
* Sets the initial angle that the quad is sitting at
* in the goal of the gyroscope measurements being used relative to it.
* @param[in]		initialAngles 		Pointer to the inital angles struct.
*/
void Pid_CalibrateInitialAngles(InitialAngles_t *InitialAngles);

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
