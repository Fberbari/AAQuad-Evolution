#ifndef _CALIBRATION_H
#define _CALIBRATION_H


#include "Common.h"
/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define DO_PILOT_CALIBRATION
#define DO_GYRO_CALIBRATION	
//#define DO_ACC_CALIBRATION

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Should be called once before the main control loop.
* Based on the presence of a definition of DO_PILOT_CALIBRATION, DO_GYRO_CALIBRATION and DO_ACC_CALIBRATION,
* will Calibrate the Sensors and PilotInstructions modules to attempt to get rid of systematic error.
* This function will also set the intial angle of the AAQuad
*/
void Calibration_Calibrate(void);

#endif // _CALIBRATION_H

// TODO, the values obtained by this module should be stored into the on chip eeprom