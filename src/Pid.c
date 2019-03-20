#include "Pid.h"

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float bestGuessXAngle;
static float bestGuessYAngle;

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


void Pid_Init(void)
{
	bestGuessXAngle = 0;
	bestGuessYAngle = 0;
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


	float xAngleFromGyro = bestGuessXAngle + (CTRL_LOOP_PERIOD * SensorResults->xGyroRate / SensorResults->nSamples);
	float yAngleFromGyro = bestGuessYAngle + (CTRL_LOOP_PERIOD * SensorResults->yGyroRate / SensorResults->nSamples);

	float xAngleFromAcc = SensorResults->xAccAngle / SensorResults->nSamples;
	float yAngleFromAcc = SensorResults->yAccAngle / SensorResults->nSamples;


	bestGuessXAngle = (0.90 * xAngleFromGyro + 0.1 * xAngleFromAcc);
	bestGuessYAngle = (0.90 * yAngleFromGyro + 0.1 * yAngleFromAcc);


	float xAdjustement = (X_SENSOR_SENSITIVITY) * bestGuessXAngle / 90.0f ;
	float yAdjustement = (Y_SENSOR_SENSITIVITY) * bestGuessYAngle / 90.0f;

	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;
/*
	motors[0] += zAdjustement;
	motors[1] -= zAdjustement;
	motors[2] += zAdjustement;
	motors[3] -= zAdjustement;
*/
	return AAQUAD_SUCCEEDED;
}

void Pid_CalibrateInitialAngles(InitialAngles_t *InitialAngles)
{
	bestGuessXAngle = InitialAngles->x;
	bestGuessYAngle = InitialAngles->y;
}