#include "Pid.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


void Pid_Init(void)
{

}


int Pid_Compute(PilotResult_t *PilotResult, SensorResults_t *SensorResults, float *motors)
{
	float xAdjustement = (MAX_X_THROW / 100.0f) * PilotResult->xPercentage;
	float yAdjustement = (MAX_Y_THROW / 100.0f) * PilotResult->yPercentage;
	float zAdjustement = (MAX_Z_THROW / 100.0f) * PilotResult->zPercentage;

	for (int i = 0; i < 4; i++)
	{
		motors[i] = PilotResult->throttlePercentage;   
	}

	motors[0] += xAdjustement;
	motors[2] -= xAdjustement;

	motors[1] += yAdjustement;
	motors[3] -= yAdjustement;

	motors[0] += zAdjustement;
	motors[1] -= zAdjustement;
	motors[2] += zAdjustement;
	motors[3] -= zAdjustement;

	return AAQUAD_SUCCEEDED;
}