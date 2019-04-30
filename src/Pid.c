#include "Pid.h"
#include "LowPassFilter.h"
#include <math.h>

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

#define MAX_VALUE_NO_PROP_SPIN	16
#define FILTER_WINDOW_SIZE 	3

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float bestGuessXAngle;
static float bestGuessYAngle;

static float xErrorArray[5];
static float yErrorArray[5];

static float previousXAdjustement;
static float previousYAdjustement;

LowPassFilter_t GyroXFilter;
LowPassFilter_t GyroYFilter;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void UpdateXErrorArray(float currentXError);
static void UpdateYErrorArray(float currentYError);

static float XProportional(void);
static float XDifferential(void);
static float XIntegral(void);

static float YProportional(void);
static float YDifferential(void);
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

	previousYAdjustement = 0;
	previousYAdjustement = 0;

	LowPassFilter_Init();

	GyroXFilter = LowPassFilter_CreateFilter(FILTER_WINDOW_SIZE);
	GyroYFilter = LowPassFilter_CreateFilter(FILTER_WINDOW_SIZE);

}

int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors)
{

	if (PilotResult->throttlePercentage < 12)	// TODO make this 12 a define
	{
		for (int i = 0; i < 4; i++)
		{
			motors[i] = 0;   
		}

		return AAQUAD_SUCCEEDED;
	}

	float filteredXGyro = LowPassFilter_Execute(GyroXFilter, (SensorResults->xGyroRate / SensorResults->nSamples));
	float filteredYGyro = LowPassFilter_Execute(GyroYFilter, (SensorResults->yGyroRate / SensorResults->nSamples));

	for (int i = 0; i < 4; i++)
	{
		motors[i] = PilotResult->throttlePercentage;   
	}

	float xAngleFromGyro = bestGuessXAngle + (CTRL_LOOP_PERIOD * filteredXGyro);
	float yAngleFromGyro = bestGuessYAngle + (CTRL_LOOP_PERIOD * filteredYGyro);
	float xAngleFromAcc = SensorResults->xAccAngle / SensorResults->nSamples;
	float yAngleFromAcc = SensorResults->yAccAngle / SensorResults->nSamples;
	bestGuessXAngle = (0.95f * xAngleFromGyro + 0.05f * xAngleFromAcc);
	bestGuessYAngle = (0.95f * yAngleFromGyro + 0.05f * yAngleFromAcc);

	float targetXAngle = (MAX_X_THROW / 100.0f) * PilotResult->xPercentage;
	float targetYAngle = (MAX_Y_THROW / 100.0f) * PilotResult->yPercentage;

	UpdateXErrorArray(targetXAngle - bestGuessXAngle);
	UpdateYErrorArray(targetYAngle - bestGuessYAngle);

	float kp = - 0.12f;
	float kd = - 0.025f;
	float ki = - 0.01f;

	/*
	float xAdjustement = -( 0.4f * XProportional() - 0.1f * XDifferential() + 0.02f * XIntegral() ) / 4.0f;
	float yAdjustement = -( 0.4f * YProportional() - 0.1f * YDifferential() + 0.02f * YIntegral() ) / 4.0f;
	float zAdjustement = 0.3f * PilotResult->zPercentage;
	*/

	float yAdjustement = previousYAdjustement + (((kp + (kd / CTRL_LOOP_PERIOD)) * yErrorArray[0]) - ((kp + (2 * kd / CTRL_LOOP_PERIOD)) * yErrorArray[1]) + ((kd / CTRL_LOOP_PERIOD) * yErrorArray[2])) + ki * YIntegral();
	previousYAdjustement = yAdjustement - ( ki * YIntegral());

	float xAdjustement = 0;
	float zAdjustement = 0;


	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;
	
	motors[0] += zAdjustement;
	motors[1] -= zAdjustement;
	motors[2] += zAdjustement;
	motors[3] -= zAdjustement;

	motors[0] = 10;
	motors[2] = 10;

	ConstrainMotorRanges(motors);
	
	return AAQUAD_SUCCEEDED;
}

void Pid_CalibrateInitialAngles(InitialAngles_t *InitialAngles)
{
	bestGuessXAngle = InitialAngles->x;
	bestGuessYAngle = InitialAngles->y;
}


static float XProportional(void)
{
	float proportional = (0.6f * xErrorArray[0] + 0.4f * xErrorArray[1] + 0.2f * xErrorArray[2] - 0.2f * xErrorArray[4] );

	return proportional;
}

static float YProportional(void)
{
	float proportional = (0.6f * yErrorArray[0] + 0.4f * yErrorArray[1] + 0.2f * yErrorArray[2] - 0.2f * yErrorArray[4] );

	return proportional;
}

static float XDifferential(void)
{
	float differential = ( (-3.0f * xErrorArray[0] + 4.0f * xErrorArray[1] - 1.0f * xErrorArray[2]) / (2.0f * CTRL_LOOP_PERIOD) );
	
	return differential;
}

static float YDifferential(void)
{
	float differential = ( (-3.0f * yErrorArray[0] + 4.0f * yErrorArray[1] - 1.0f * yErrorArray[2]) / (2.0f * CTRL_LOOP_PERIOD) );
	
	return differential;
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
	if (currentIntegral > 30.0f)	// check for windup
	{
		if (nextSlice > 0.0f)
		{
			return true;
		}
	}

	else if (currentIntegral < - 30.0f)
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