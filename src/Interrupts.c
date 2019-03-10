
#include <avr/io.h>
#include <avr/interrupt.h>

#include "PilotInstructions.h"


ISR(INT0_vect)		// aileron
{
	
	uint16_t timeStamp = TCNT1;
	PilotInstructions_SetYTimestamp(timeStamp);	
}

ISR(INT1_vect)		// throttle
{
	
	uint16_t timeStamp = TCNT1;
	PilotInstructions_SetThrottleTimestamp(timeStamp);	
}

ISR(PCINT1_vect)	// rudder
{
	uint16_t timeStamp = TCNT1;
	PilotInstructions_SetZTimestamp(timeStamp);	
}

ISR(PCINT2_vect)	// elevator
{
	uint16_t timeStamp = TCNT1;
	PilotInstructions_SetXTimestamp(timeStamp);	
}