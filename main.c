/*
 * AAQuad-Evolution.cpp
 *
 * Created: 2019-02-24 7:15:01 PM
 * Author : Anthony Berbari
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "PilotInstructions.h"
#include "Common.h"

static volatile int a;

int main(void)
{	
	// external int 0
	EIMSK |= (1 << INT1); // enable the int0 interrupt
	EICRA |= (1 << ISC10);	// will fire at any logical change
	
	// set the 16 bit timer to normal mode
	TCCR1B = (1 << CS11);	// 8x preScaler
	
	sei();

	PilotResult_t PilotResult = {0};

	PilotInstructions_Init();
	
    while (1) 
    {
	
		int r = PilotInstructions_ComputePilotResult(&PilotResult);
		if (r != AAQUAD_BUSY)
		{
			volatile int crap;
			crap ++;
		}
    }
}