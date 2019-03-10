#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>
#include <stdbool.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define F_CPU 8000000UL



#define AAQUAD_BUSY 2
#define AAQUAD_SUCCEEDED 1
#define AAQUAD_FAILED 0 


#define TIMER_1_PRESCALER	8U


typedef struct
{
	float xPercentage;			// a number between -100 and 100 that represents the Elevator channel
	float yPercentage;			// a number between -100 and 100 that represents the Aileron channel
	float zPercentage;			// a number between -100 and 100 that represents the Rudder Channel
	float throttlePercentage;		// a number between 0 and 100 that represents the Throttle channel

}PilotResult_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput);

#endif // _COMMON_H
