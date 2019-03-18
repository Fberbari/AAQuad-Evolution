#ifndef _CALIBRATION_H
#define _CALIBRATION_H


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
void Calibration_Init(void);

/**
* Samples the sensor and recalibrates there 0 values. 
* Will pass these values to the SensorData module when Finished.
* This is a blocking call, should be called once before using any of the sensor's data.
* Note: The quad must be level during the duration of this function.
*/
void Calibration_Calibrate(void);

#endif // _CALIBRATION_H
