#ifndef _COMMON_H
#define _COMMON_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define F_CPU 20000000UL

#include <util/delay.h>

#define AAQUAD_BUSY 		2
#define AAQUAD_SUCCEEDED	1
#define AAQUAD_FAILED 		0 

#define CTRL_LOOP_PERIOD	0.005f	// in seconds
#define ALTITUDE_REFRESH_PERIOD 0.05f // in seconds

#define MAX_VALUE_NO_PROP_SPIN	12.0f
#define MOTOR_VALUE_NO_SPIN		0.5f 	// 0 should not be used as a small electrical glitch may produce an undefined (negative) signal and confuse the esc's

#define RAD_TO_DEGREE 57.3f

typedef struct
{
	float xPercentage;			// a number between -100 and 100 that represents the Elevator channel
	float yPercentage;			// a number between -100 and 100 that represents the Aileron channel
	float zPercentage;			// a number between -100 and 100 that represents the Rudder Channel
	float throttlePercentage;	// a number between 0 and 100 that represents the Throttle channel

}PilotResult_t;

typedef struct ImuData
{
	float gyrX, gyrY, gyrZ;	// data given in rad/s
	float accX, accY, accZ;	// both mag and acc are arbitrary units
	float magX, magY, magZ;	// On the current pcb, when looking straight down onto it, x is in the forward direction, y is to the left, and z is up towards your face.

}ImuData_t;

typedef struct EulerXYZ
{
    float phi;
    float theta;
    float psi;

}EulerXYZ_t;

typedef struct EulerRates
{
    float phiDot;
    float thetaDot;
    float psiDot;

}EulerRates_t;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput);

float Square(int16_t num);

float Squaref(float num);

float SignedSquaref(float num);

/**
* This function converts a quaternion to the equivalent XYZ euler angles.
* All angles are given in radians.
* param[in]			q0,q1,q2,q3			the quaternion we wish to convert to euler angles
* @param[out]		EulerAngles 		pointer to the result, in radians
*/
void quat2Euler(float q0, float q1, float q2, float q3, EulerXYZ_t *EulerAngles);

/**
* Given the current orientation of an object, and it's angular velocities, this function gives the derivatives of the XYZ euler angles.
* param[in]			angVelX, angVelY, angVelZ		The angular velocities of the quad, in rad/s as reported by an onboard gyroscope.
* param[in]			EulerAngles						The orientation of the quad, in radians.
* @param[out]		EulerRates 						The derivatives of the euler angles, in rad/s
*/
void gyro2EulerRates(EulerXYZ_t *EulerAngles, float angVelX, float angVelY, float angVelZ, EulerRates_t *EulerRates);

#endif // _COMMON_H
