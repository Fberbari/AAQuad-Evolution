#ifndef _CALIBRATION_H
#define _CALIBRATION_H


#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* If pilot rquests a Calibration (with a specific sequenceon is radio, see README), 
* all current sensor output and pilot output readings are stored into EEPROM.
* Before exiting, will load the most recent calibration from EEPROM and deliver it to the modules that require it.
*/
void Calibration_Calibrate(void);

#endif // _CALIBRATION_H

// TODO, the values obtained by this module should be stored into the on chip eeprom