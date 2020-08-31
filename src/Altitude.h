#ifndef _ALTITUDE_H
#define _ALTITUDE_H

#include "Common.h"

/**********************************************************
 * Prototypes
 *********************************************************/

/**
* Initializes internal parameters.
* Should be called exactly once before anything is attempted to be done with the module.
*/
void Altitude_Init(void);

/**
* Calibrates the ultrasonic distance sensor.
* @param[in]	ultrasonicCalibration 	systematic error associated with collection of the altitude measurement from the ultrasonic sensor.
* @param[in]	altimeterCalibration 	systematic error associated with collection of the altitude measurement from the altimeter.
*/
void Altitude_LoadCalibration(float ultrasonicCalibration, float altimeterCalibration);

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
* This data is accurate to the centimeter in the 0-3 m range and accurate to the meter above that.
* Returns succeeded if the data was successfully collected, busy if new data is not yet available, failed otherwise.
* @param[out]		altitude 		pointer to the result, in meters
* @return			AAQUAD_SUCCEEDED, AAQUAD_BUSY or AAQUAD_FAILED
*/
int Altitude_Get(float *altitude);

/**
* THESE FUNCTIONS ARE NOT MEANT FOR USE IN THE CONTROL LOOP.
* For use in the control loop, use the above "get" function. That one internally fuses altimeter and ultrasonic distance sensor data.
* These functions are exposed for use with the calibration module.
* @param[out]		altitude 		pointer to the result, as reported from the given sensor, in meters.
* @return			AAQUAD_SUCCEEDED, AAQUAD_BUSY or AAQUAD_FAILED
*/
int Altitude_AltimeterGet(float *altitude);
int Altitude_UltrasonicGet(float *altitude);

#endif
