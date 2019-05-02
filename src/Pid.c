#include "Pid.h"
#include <math.h>

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

#define MAX_VALUE_NO_PROP_SPIN	12

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float bestGuessXAngle;
static float bestGuessYAngle;
static float bestGuessZRate;

static float xErrorArray[5];
static float yErrorArray[5];
static float zErrorArray[5];

static float previousXAdjustement;
static float previousYAdjustement;
static float previousZAdjustement;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void UpdateXErrorArray(float currentXError);
static void UpdateYErrorArray(float currentYError);
static void UpdateZErrorArray(float currentZError);

static float XIntegral(void);

static float YIntegral(void);

static void ConstrainMotorRanges(float *motors);
static bool IntegralWillWindUp(float currentIntegral, float nextSlice);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


void Pid_Init(void)
{
	bestGuessXAngle = 0;
	bestGuessYAngle = 0;

	for (int i = 0; i < 5; i++)
	{
		xErrorArray[i] = 0;
		yErrorArray[i] = 0;
	}

	previousXAdjustement = 0;
	previousYAdjustement = 0;

}

int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors)
{

	if (PilotResult->throttlePercentage < MAX_VALUE_NO_PROP_SPIN)
	{
		for (int i = 0; i < 4; i++)
		{
			motors[i] = 0;   
		}

		return AAQUAD_SUCCEEDED;
	}

	for (int i = 0; i < 4; i++)
	{
		motors[i] = PilotResult->throttlePercentage;   
	}

	float xAngleFromGyro = bestGuessXAngle + (CTRL_LOOP_PERIOD * SensorResults->xGyroRate);
	float yAngleFromGyro = bestGuessYAngle + (CTRL_LOOP_PERIOD * SensorResults->yGyroRate);
	float xAngleFromAcc = SensorResults->xAccAngle;
	float yAngleFromAcc = SensorResults->yAccAngle;
	bestGuessXAngle = (0.98f * xAngleFromGyro + 0.02f * xAngleFromAcc);
	bestGuessYAngle = (0.98f * yAngleFromGyro + 0.02f * yAngleFromAcc);
	bestGuessZRate = SensorResults->zGyroRate;

	float targetXAngle = (MAX_X_THROW / 100.0f) * PilotResult->xPercentage;
	float targetYAngle = (MAX_Y_THROW / 100.0f) * PilotResult->yPercentage;
	float targetZRate = (MAX_Z_THROW / 100.0f) * PilotResult->zPercentage;

	UpdateXErrorArray(bestGuessXAngle - targetXAngle);
	UpdateYErrorArray(bestGuessYAngle - targetYAngle);
	UpdateZErrorArray(bestGuessZRate - targetZRate);

	float xykp = 0.125f;
	float xykd = 0.025f;

	float zkp = 0.1;
	float zkd = 0.015;

	float xAdjustement = previousXAdjustement + (((xykp + (xykd / CTRL_LOOP_PERIOD)) * xErrorArray[0]) - ((xykp + (2 * xykd / CTRL_LOOP_PERIOD)) * xErrorArray[1]) + ((xykd / CTRL_LOOP_PERIOD) * xErrorArray[2]));
	previousXAdjustement = xAdjustement;
	
	float yAdjustement = previousYAdjustement + (((xykp + (xykd / CTRL_LOOP_PERIOD)) * yErrorArray[0]) - ((xykp + (2 * xykd / CTRL_LOOP_PERIOD)) * yErrorArray[1]) + ((xykd / CTRL_LOOP_PERIOD) * yErrorArray[2]));
	previousYAdjustement = yAdjustement;

	float zAdjustement = previousZAdjustement + (((zkp + (zkd / CTRL_LOOP_PERIOD)) * zErrorArray[0]) - ((zkp + (2 * zkd / CTRL_LOOP_PERIOD)) * zErrorArray[1]) + ((zkd / CTRL_LOOP_PERIOD) * zErrorArray[2]));
	previousZAdjustement = zAdjustement;



	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;
	
	motors[0] -= zAdjustement;
	motors[1] += zAdjustement;
	motors[2] -= zAdjustement;
	motors[3] += zAdjustement;

	ConstrainMotorRanges(motors);
	
	return AAQUAD_SUCCEEDED;
}

void Pid_CalibrateInitialAngles(InitialAngles_t *InitialAngles)
{
	bestGuessXAngle = InitialAngles->x;
	bestGuessYAngle = InitialAngles->y;
}

static float XIntegral(void)
{
	static float integral;
	
	float slice = (( xErrorArray[0] + 4.0f * xErrorArray[1] + xErrorArray[2] )* 0.833f * CTRL_LOOP_PERIOD);
	
	if ( ! IntegralWillWindUp(integral, slice))
	{
		integral += slice;
	}

	return integral;
}

static float YIntegral(void)
{
	static float integral;

	float slice = (0.6f * yErrorArray[0] + 0.4f * yErrorArray[1] + 0.2f * yErrorArray[2] - 0.2f * yErrorArray[4] );
	
	if ( ! IntegralWillWindUp(integral, slice))
	{
		integral += slice;
	}

	return integral;
}


static bool IntegralWillWindUp(float currentIntegral, float nextSlice)
{
	if (currentIntegral > 40.0f)	// check for windup
	{
		if (nextSlice > 0.0f)
		{
			return true;
		}
	}

	else if (currentIntegral < - 40.0f)
	{
		if (nextSlice < 0.0f)
		{
			return true;
		}
	}
	
	return false;
}




static void UpdateXErrorArray(float currentXError)
{
	for (int i = 4; i > 0; i-- )
	{
		xErrorArray[i] = xErrorArray[i-1];
	}

	xErrorArray[0] = currentXError;
}

static void UpdateYErrorArray(float currentYError)
{
	for (int i = 4; i > 0; i-- )
	{
		yErrorArray[i] = yErrorArray[i-1];
	}

	yErrorArray[0] = currentYError;
}

static void UpdateZErrorArray(float currentZError)
{
	for (int i = 4; i > 0; i-- )
	{
		zErrorArray[i] = zErrorArray[i-1];
	}

	zErrorArray[0] = currentZError;
}

static void ConstrainMotorRanges(float *motors)
{
	if (motors[0] > 100)
	{
		motors[0] = 100;
	}
	else if (motors[0] < 0)
	{
		motors[0] = 0;
	}
	
	if (motors[1] > 100)
	{
		motors[1] = 100;
	}
	else if (motors[1] < 0)
	{
		motors[1] = 0;
	}
	
	if (motors[2] > 100)
	{
		motors[2] = 100;
	}
	else if (motors[2] < 0)
	{
		motors[2] = 0;
	}
	
	if (motors[3] > 100)
	{
		motors[3] = 100;
	}
	else if (motors[3] < 0)
	{
		motors[3] = 0;
	}
}