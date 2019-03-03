#ifndef _PILOTINSTRUCTIONS_H
#define _PILOTINSTRUCTIONS_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void PilotInstructions_Init(void);

/**
* Executes a cycle of the module.
* Returns AAQUAD_SUCCEEDED if new commands are available.
* Returns AAQUAD_BUSY if data is being processed.
* Returns AAQUAD_FAILED if the module failed internally.
* @return 			AAQUAD_SUCCEEDED, AAQUAD_BUSY, AAQUAD_FAILED
*/
int PilotInstructions_Do(void);

/**
* Gives the latest information read from the receiver, if it is available.
* Will only give valid data if all 4 receiver channels were updated after the last time the user called this function.
* If this function is called before new data is available, the isUpdated field of the return struct will be false.
* @param[out]	PilotResult 		A pointer the PilotInstructions struct.
*/
void PilotInstructions_GetPilotResult(PilotResult_t *PilotResult);



/**
* These functions are meant to be called by the appropriate ISR.
* They will update the module's internal data to the data provided.
* @param[in]	Timestamp 		The value of a 16 bit counter at the moment data was recorded.
*/
void PilotInstructions_SetXTimestamp(uint16_t xTimestamp);
void PilotInstructions_SetYTimestamp(uint16_t yTimestamp);
void PilotInstructions_SetZTimestamp(uint16_t zTimestamp);
void PilotInstructions_SetThrottleTimestamp(uint16_t throttleTimestamp);


#endif // _PILOTINSTRUCTIONS_H
