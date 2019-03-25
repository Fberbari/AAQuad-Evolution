#include "Pid.h"

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float bestGuessXAngle;
static float bestGuessYAngle;

static float xErrorArray[5];
static float yErrorArray[5];

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
}


int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors)
{

	for (int i = 0; i < 4; i++)
	{
		motors[i] = PilotResult->throttlePercentage;   
	}
	

	float xAngleFromGyro = bestGuessXAngle + (CTRL_LOOP_PERIOD * SensorResults->xGyroRate / SensorResults->nSamples);
	float yAngleFromGyro = bestGuessYAngle + (CTRL_LOOP_PERIOD * SensorResults->yGyroRate / SensorResults->nSamples);
	float xAngleFromAcc = SensorResults->xAccAngle / SensorResults->nSamples;
	float yAngleFromAcc = SensorResults->yAccAngle / SensorResults->nSamples;
	bestGuessXAngle = (0.95 * xAngleFromGyro + 0.05 * xAngleFromAcc);
	bestGuessYAngle = (0.95 * yAngleFromGyro + 0.05 * yAngleFromAcc);

	float targetXAngle = (MAX_X_THROW / 100.0f) * PilotResult->xPercentage;
	float targetYAngle = (MAX_Y_THROW / 100.0f) * PilotResult->yPercentage;

	UpdateXErrorArray(targetXAngle - bestGuessXAngle);
	UpdateYErrorArray(targetYAngle - bestGuessYAngle);


	float xAdjustement = ( 0.3 * XProportional() - 0.25 * XDifferential() + 0.05 * XIntegral() ) / 8.0f;
	float yAdjustement = ( 0.3 * YProportional() - 0.25 * YDifferential() + 0.05 * YIntegral() ) / 8.0f;
	float zAdjustement = 0.3 * PilotResult->zPercentage;


	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;

	motors[0] += zAdjustement;
	motors[1] -= zAdjustement;
	motors[2] += zAdjustement;
	motors[3] -= zAdjustement;

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

	integral += ( xErrorArray[0] + (4 * xErrorArray[1]) + xErrorArray[2] )* 0.833 * CTRL_LOOP_PERIOD;

	return integral;
}

static float YIntegral(void)
{
	static float integral;

	integral += ( yErrorArray[0] + (4 * yErrorArray[1]) + yErrorArray[2] )* 0.833 * CTRL_LOOP_PERIOD;

	return integral;
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