#ifndef _SENSOR_DATA_H
#define _SENSOR_DATA_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/
/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void SensorData_Init(void);

/**
* Should be called once right after Init.
* Uses the values of the members of SensorResuts to set a 0 baseline to remove systematic error associated.
* If it is not called, the module will assume there are no errors.
* @param[in]	CalibratedZeros 		struct that contains actual values of the Sensor channels that correspond to a level Quad position with no motion in any axis.
*/
void SensorData_LoadCalibration(SensorResults_t *CalibratedZeros);

/**
* Gives the Initial Angle of the Quad.
* This result of this function is only valid if it is called when the quad is motionless in every axis.
* @ param[out]		initialXAngle		The intial x Angle of the quadcopter
* @ param[out]		initialYAngle		The intial y Angle of the quadcopter
*/
void SensorData_GetInitialAngles(float *initialXAngle, float *initialYAngle);

/**
* Gets the information about the quad's orientation.
* This is a blocking function that will not return until communication with the sensors either completed or failed.
* Returns succeeded if the data was successfully collected and computed, returns failed otherwise.
* @param[in & out]		SensorResults 		pointer to the results struct
* @return			AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int SensorData_GetResult(SensorResults_t *SensorResults);

#endif // _SENSOR_DATA_H
