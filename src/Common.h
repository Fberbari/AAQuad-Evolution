#ifndef _COMMON_H
#define _COMMON_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define F_CPU 20000000UL



#define AAQUAD_BUSY 		2
#define AAQUAD_SUCCEEDED	1
#define AAQUAD_FAILED 		0 

#define CTRL_LOOP_PERIOD	0.021f	// in seconds

#define TIMER_1_PRESCALER	8U

#define MAX_VALUE_NO_PROP_SPIN	12.0f
#define MOTOR_VALUE_NO_SPIN		2.0f 	// 0 should not be used as a small electrical glitc may produce an undefined (negative) signal and confuse the esc's

typedef struct
{
	float xPercentage;			// a number between -100 and 100 that represents the Elevator channel
	float yPercentage;			// a number between -100 and 100 that represents the Aileron channel
	float zPercentage;			// a number between -100 and 100 that represents the Rudder Channel
	float throttlePercentage;	// a number between 0 and 100 that represents the Throttle channel

}PilotResult_t;

typedef struct 
{
	float xAccAngle;				// pitch angle in degrees (positive is when motor 0 lowers, 0 is level)
	float yAccAngle;				// bank angle in degrees (positive is when motor 3 lowers, 0 is level)

	float xGyroRate;				// rotation speed in degrees per second (positive is motor 2 dipping down and motor 0 going up)
	float yGyroRate;				// rotation speed in degrees per second (positive is motor 3 dipping down and motor 1 going up)
	float zGyroRate;				// rotation speed in degrees per second (positive is clockwise when looking down at the quad)
	
}SensorResults_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput);

float Square(int16_t num);

#endif // _COMMON_H
