
#include <avr/io.h>
#include <avr/interrupt.h>

#include "PilotInstructions.h"



ISR(INT1_vect){
	
	uint16_t timeStamp = TCNT1;
	PilotInstructions_SetThrottleTimestamp(timeStamp);	
}