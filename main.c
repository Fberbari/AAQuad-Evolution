/*
 * AAQuad-Evolution.cpp
 *
 * Created: 2019-02-24 7:15:01 PM
 * Author : Anthony Berbari
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "I2CDriver.h"
#include "Controller.h"
#include "Calibration.h"
#include <stdlib.h>

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
	Calibration_Init();


	Calibration_Calibrate();

	
    while (1) 
    {
    	Controller_Do();
    }
	
}


static void InitPeripherals(void)
{

	I2CDriver_Init();

	// aileron
	EIMSK |= (1 << INT0); // enable the int0 interrupt												
	EICRA |= (1 << ISC00);	// will fire at any logical change

	// throttle
	EIMSK |= (1 << INT1); // enable the int1 interrupt
	EICRA |= (1 << ISC10);	// will fire at any logical change

	// rudder
	PCICR |= (1 << PCIE1);	// enable pcint 1
	PCMSK1 |= (1 << PCINT11);

	//elevator
	PCICR |= (1 << PCIE2);	// enable pcint 2
	PCMSK2 |= (1 << PCINT17);
	
	// set the 16 bit timer to normal mode
	TCCR1B = (1 << CS11);	// 8x preScaler

	sei();
}

