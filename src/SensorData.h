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
* gets the information about the quad's orientation
* This is a blocking function that will not return until communication with the sensors either completed or failed.
* Returns succeeded if the data was successfully brought in and computed, returns failed otherwise.
* @param[out]		SensorResults 		pointer to the results struct
* @return			AAQUAD_SUCCEEDED or AAQUAD_FAILED
*/
int SensorData_GetResult(SensorResults_t *SensorResults);


#endif // _SENSOR_DATA_H
