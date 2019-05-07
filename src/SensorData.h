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
* Gives the Initial Angle of the Quad.
* This result is this function is called when the quad is motionless in every axis.
* @ param[out]		initialXAngle		The intial x Angle of the qudcopter
* @ param[out]		initialYAngle		The intial y Angle of the qudcopter
*/
void SensorData_GetInitialAngles(float *initialXAngle, float *initialYAngle);

/**
* Zeros the current Gyroscope Data.
* Calling this function is optional.
* If it is not called, any systematic error in the gyro Data will remain.
* If it is called, it should be ensured that at the time of calling, the Quad is not moving in any axis.
*/
void SensorData_CalibrateGyro(void);

/**
* Zeros the current Accelerometer Data.
* Calling this function is optional.
* If it is not called, any systematic error in the Accelerometer Data will remain.
* If it is called, it should be ensured that at the time of calling, the Quad is not moving in
* any axis and pointing straight up, so that it can be set to 1g on the z axis and 0g's on the x and y.
*/
void SensorData_CalibrateAcc(void);


/**
* Gets and accumulates the information about the quad's orientation.
* Will update the given struct with the info without clearing it, so make sure the first call of this function is with a 0'd struct.
* This is a blocking function that will not return until communication with the sensors either completed or failed.
* Returns succeeded if the data was successfully brought in and computed, returns failed otherwise.
* @param[in & out]		SensorResults 		pointer to the results struct
* @return			AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int SensorData_GetResult(SensorResults_t *SensorResults);

#endif // _SENSOR_DATA_H
