#ifndef _SENSOR_DATA_H
#define _SENSOR_DATA_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

 typedef struct 
 {
	float xGyroRate;				
	float yGyroRate;
	float zGyroRate;

 }CalibratedZeros_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/
/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void SensorData_Init(void);

/**
* Meant To Be called by Calibration.
* Sets the adjustements to be given to every measurement to compensate for sensor drift over time.
* @param[in]		CalibratedZeros 		Pointer to the struct of zero values.
*/
void SensorData_CalibrateZeros(CalibratedZeros_t *Zeros);

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
