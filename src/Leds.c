#include "Leds.h"
#include <avr/io.h>


/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Leds_Init(void)
{
	DDRD |= ((1 << 5) | (1 << 6));
}

void Leds_SetLed0(void)
{
	PORTD |= (1 << 5);
}

void Leds_ClearLed0(void)
{
	PORTD &= ~(1 << 5);
}

void Leds_ToggleLed0(void)
{
	PORTD ^= (1 << 5);
}

void Leds_SetLed1(void)
{
	PORTD |= (1 << 6);
}

void Leds_ClearLed1(void)
{
	PORTD &= ~(1 << 6);
}

void Leds_ToggleLed1(void)
{
	PORTD ^= (1 << 6);
}
