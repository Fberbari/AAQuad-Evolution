#include "Calibration.h"
#include "SensorData.h"
#include "PilotInstructions.h"
#include "Pid.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define DO_PILOT_CALIBRATION
#define DO_GYRO_CALIBRATION

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Calibration_Calibrate(void)
{

#ifdef	DO_GYRO_CALIBRATION	
	SensorData_CalibrateGyro();
#endif


#ifdef 	DO_ACC_CALIBRATION
	SensorData_CalibrateAcc();
#endif

#ifdef DO_PILOT_CALIBRATION
	PilotInstructions_Calibrate();
#endif

	float initialXAngle = 0.0f;
	float initialYAngle = 0.0f;

	SensorData_GetInitialAngles(&initialXAngle, &initialYAngle);
	Pid_SetIntialAngles(initialXAngle, initialYAngle);

}