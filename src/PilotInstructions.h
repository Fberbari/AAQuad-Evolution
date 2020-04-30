#ifndef _PILOTINSTRUCTIONS_H
#define _PILOTINSTRUCTIONS_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters.
* Should be called exactly once before anything is attempted to be done with the module.
*/
void PilotInstructions_Init(void);

/**
* Should called once right after Init.
* Uses the values of the members of pilot result to set a 0 baseline to remove systematic error.
* If it is not called, the module will assume there are no errors.
* @param[in]	Calibration 		struct that contains actual values of the pilot channels that correspond to a pilot intention of 0.
*/
void PilotInstructions_LoadCalibration(PilotResult_t *Calibration);

/**
* Gathers and processes the latest information read from the receiver, if it is available.
* Will retun AAQUAD_SUCCEEDED if all 4 receiver channels were updated after
* the last time the user called this function and the data was valid.
* Will return AAQUAD_BUSY if all 4 channels are not updated yet.
* Will return AAQUAD_FAILED in case of any internal failure.
* The contents of the [out] parameter are only modified in the case of a successful return.
* @param[out]	PilotResult 		A pointer the PilotInstructions struct.
* @return 		AAQUAD_SUCCEEDED, AAQUAD_BUSY, AAQUAD_FAILED
*/
int PilotInstructions_ComputePilotResult(PilotResult_t *PilotResult);

#endif // _PILOTINSTRUCTIONS_H
