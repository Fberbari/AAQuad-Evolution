#include "Leds.h"
#include <avr/io.h>


/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Leds_Init(void)
{
	DDRB |= ((1 << 1) | (1 << 0));
}

void Leds_SetLed0(void)
{
	PORTB |= (1 << 0);
}

void Leds_ClearLed0(void)
{
	PORTB &= ~(1 << 0);
}

void Leds_ToggleLed0(void)
{
	PORTB ^= (1 << 0);
}

void Leds_SetLed1(void)
{
	PORTB |= (1 << 1);
}

void Leds_ClearLed1(void)
{
	PORTB &= ~(1 << 1);
}

void Leds_ToggleLed1(void)
{
	PORTB ^= (1 << 1);
}
