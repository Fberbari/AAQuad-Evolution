#include "Pid.h"

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float xAngleFromGyro;
static float yAngleFromGyro;

static float xAngleFromAcc;
static float yAngleFromAcc;

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


void Pid_Init(void)
{
}


int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors)
{

	for (int i = 0; i < 4; i++)
	{
		motors[i] = PilotResult->throttlePercentage;   
	}
	
	//float xAdjustement = (MAX_X_THROW / 100.0f) * PilotResult->xPercentage;
	//float yAdjustement = (MAX_Y_THROW / 100.0f) * PilotResult->yPercentage;
	//float zAdjustement = (MAX_Z_THROW / 100.0f) * PilotResult->zPercentage;

	//float xAdjustement = ((X_SENSOR_SENSITIVITY) * (SensorResults->xAngle / 90.0f) / SensorResults->nSamples);
	//float yAdjustement = ((Y_SENSOR_SENSITIVITY) * (SensorResults->yAngle / 90.0f) / SensorResults->nSamples);

	xAngleFromGyro += CTRL_LOOP_PERIOD * SensorResults->xGyroRate / SensorResults->nSamples;
	yAngleFromGyro += CTRL_LOOP_PERIOD * SensorResults->yGyroRate / SensorResults->nSamples;

	xAngleFromAcc = SensorResults->xAccAngle / SensorResults->nSamples;
	yAngleFromAcc = SensorResults->yAccAngle / SensorResults->nSamples;


	float bestGuessXAngle = (0.98 * xAngleFromGyro + 0.02 * xAngleFromAcc);
	float bestGuessYAngle = (0.98 * yAngleFromGyro + 0.02 * yAngleFromAcc);


	float xAdjustement = (X_SENSOR_SENSITIVITY) * bestGuessXAngle / 90.0f ;
	float yAdjustement = (Y_SENSOR_SENSITIVITY) * bestGuessYAngle / 90.0f;
/*
	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;

	motors[0] += zAdjustement;
	motors[1] -= zAdjustement;
	motors[2] += zAdjustement;
	motors[3] -= zAdjustement;
*/
	return AAQUAD_SUCCEEDED;
}

void Pid_CalibrateInitialAngles(InitialAngles_t *InitialAngles)
{
	xAngleFromGyro = InitialAngles->x;
	yAngleFromGyro = InitialAngles->y;
}