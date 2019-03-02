/*
 * AAQuad-Evolution.cpp
 *
 * Created: 2019-02-24 7:15:01 PM
 * Author : Anthony Berbari
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000U


static volatile int a;

int main(void)
{	
	// external int 0
	EIMSK |= (1 << INT0); // enable the int0 interrupt
	EICRA |= (1 << ISC00);	// will fire at any logical change
	
	sei();
	volatile int b;
	
    while (1) 
    {
		b ++;
    }
}

ISR(INT0_vect)
{
	a++;
}