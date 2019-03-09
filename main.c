/*
 * AAQuad-Evolution.cpp
 *
 * Created: 2019-02-24 7:15:01 PM
 * Author : Anthony Berbari
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Controller.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void InitPeripherals(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


int main(void)
{

	Controller_Init();
	InitPeripherals();
	
    while (1) 
    {
    	Controller_Do();
    }
	
}


static void InitPeripherals(void)
{
	// external int 0
	EIMSK |= (1 << INT1); // enable the int0 interrupt
	EICRA |= (1 << ISC10);	// will fire at any logical change
	
	// set the 16 bit timer to normal mode
	TCCR1B = (1 << CS11);	// 8x preScaler

	sei();
}

