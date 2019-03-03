#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>
#include <stdbool.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
#ifndef F_CPU
#define F_CPU 8000000
#endif


#define AAQUAD_BUSY 2
#define AAQUAD_SUCCEEDED 1
#define AAQUAD_FAILED 0 


typedef struct
{
	float xAngle;			// the requested pitch angle
	float yAngle;			// the requested bank angle
	float zRate;			// the requested yaw rate in deg/s
	float throttlePower;	// a percentage between 0 and 100

}PilotResult_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput);


#endif // _COMMON_H
