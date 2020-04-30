#ifndef _CALIBRATION_H
#define _CALIBRATION_H


#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Call this function exactly once, after all the other modules are initialised, but before the main control loop begins.
* If the pilot requests a calibration (with a specific sequence on their radio, see README), 
* the module collects sensor and pilot measurements and stores them in eeprom.
* Those measurements are then used to calibrate the appropriate modules everytime the board is powered.
*/
void Calibration_Calibrate(void);

#endif // _CALIBRATION_H
