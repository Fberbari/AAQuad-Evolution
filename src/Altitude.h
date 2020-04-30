#ifndef _ALTITUDE_H
#define _ALTITUDE_H

#include "Common.h"

/**********************************************************
 * Prototypes
 *********************************************************/

/**
* Initialises internal parameters.
* Should be called exactly once before anything is attempted to be done with the module.
*/
void Altitude_Init(void);

/**
* Calibrates the ultrasonic distance sensor.
* @param[in]	altitudeCalibration 	systematic error associated with collection of the altitude measurement.
*/
void Altitude_LoadCalibration(float altitudeCalibration);

/**
* Begins the process of measuring the altitude.
* Call this function synchronously.
* The user may call this at any rate they wish without fear of burdening the hardware, however, the sensor can only collect data at 
* roughly 20 Hz so that is how often the number will actually be updated.
* This is a non blocking function that returns right away.
*/
void Altitude_BeginMeasurement(void);

/**
* Gets the quad's altitude.
* This is a non blocking function that returns right away with the results if they are available or an
* indication that they are still being collected if they aren't.
* Returns succeeded if the data was successfully collected, busy if new data is not yet available, failed otherwise.
* @param[out]		altitude 		pointer to the result, in meters
* @return			AAQUAD_SUCCEEDED, AAQUAD_BUSY or AAQUAD_FAILED
*/
int Altitude_Get(float *altitude);

#endif
