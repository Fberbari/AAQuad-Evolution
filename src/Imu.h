#ifndef _IMU_H
#define _IMU_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called exactly once before anything is attempted to be done with the module.
*/
void Imu_Init(void);

/**
* Calibrates the IMU to compensate for any offsets and systematic error.
* Should be called after Init and before IMU data gets used by the main control loop.
* @param[in]	ImuCalibration 		struct that contains the compensation values.
*/
void Imu_LoadCalibration(ImuData_t *ImuCalibration);

/**
* Begins the process of capturing the imu data.
* Call this function synchronously.
* This is a non blocking function that returns right away.
*/
void Imu_BeginRead(void);

/**
* Gets the information about the quad's orientation.
* The magnetometer data returned by this function has NOT been compensated for temperature. TODO maybe I should ?
* This is a non blocking function that returns right away with the results if they are available or an
* indication that they are still being collected if they aren't.
* Returns succeeded if the data was successfully collected, busy if new data is not yet available, failed otherwise.
* The contents of the [out] parameter are only modified in the case of a successful return.
* Accelerometer data is given in mg (1 g being earth's acceleration) and gyroscope data is given in rad/s while
* magnetometer data is given in arbitrary units (in anticipation that the vector will be normalized anyway).
* @param[out]		imuData 		pointer to the results struct
* @return			AAQUAD_SUCCEEDED, AAQUAD_BUSY or AAQUAD_FAILED
*/
int Imu_GetResult(ImuData_t *ImuData);

#endif // _IMU_H
