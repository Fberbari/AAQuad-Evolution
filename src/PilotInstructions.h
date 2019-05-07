#ifndef _PILOTINSTRUCTIONS_H
#define _PILOTINSTRUCTIONS_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define MAX_PWM_DUTYCYCLE_S 0.002f	// max DutyCycle that the pwm signal from the receiver can have  in s
#define MIN_PWM_DUTYCYCLE_S	0.001f	// min DutyCycle that the pwm signal from the receiver can have  in s

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void PilotInstructions_Init(void);

/**
* Zeros all current pilot inputs (except throttle)
* Calling this function is optional.
* If it is not called, any systematic error in the pilot's commands will remain.
* If it is called, it should be ensured that at the time of calling, the pilot's controls are at 0 (relative to him).
* Note: If trim is used, this function should not be called, as trim on any of the channels will be zeroed.
*/
void PilotInstructions_Calibrate(void);

/**
* Gathers and processes the latest information read from the receiver, if it is available.
* Will retun AAQUAD_SUCCEEDED if all 4 receiver channels were updated after
* the last time the user called this function and the data was valid.
* Will return AAQUAD_BUSY if all 4 channels are not updated yet.
* Will return AAQUAD_FAILED in case of any internal failure.
* @param[out]	PilotResult 		A pointer the PilotInstructions struct.
* @return 		AAQUAD_SUCCEEDED, AAQUAD_BUSY, AAQUAD_FAILED
*/
int PilotInstructions_ComputePilotResult(PilotResult_t *PilotResult);



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
