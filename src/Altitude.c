#include "Altitude.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define SPEED_OF_SOUND 340.0f  // m/s

#define MAX_16_BIT_VALUE 65535U

#define SECONDS_PER_TICK ((float)(64.0f / F_CPU))    // assuming timer has presaler of 64

#define TRIG_PIN 1

#define TIMER_VAL_11_US (F_CPU  / 90000)
#if (TIMER_VAL_11_US > 255)
	#error "8 bit timer cannot count to 11uS before overflowing, CPU is too fast"
#endif

/**********************************************************
 * Variables
 *********************************************************/

static float measuredAltitude;
static bool dataReady;
static bool inProgress;

static float systematicError;

/**********************************************************
 * Code
 *********************************************************/

void Altitude_Init(void)
{
    // timer 0 used to manage the trigger pulse
    // interrupts are initialised but the counter is not started yet
	TIMSK0 = (1 << OCIE0A);
    OCR0A = TIMER_VAL_11_US;

    // timer 3 used to measure the length of the echo pulse (prescaler of 64)
    TCCR3B = ((1 << CS31) | (1 << CS30));

    DDRB |= (1 << TRIG_PIN);	// TODO I burned the output driver of the trig pin so the temporary solution is to swap the trig and echo pin. This is currently done in this file. Just do it on the pcb. Permanent fix is just to replace the microcontroller.
    PORTB &= ~(1 << TRIG_PIN);

    // pcint for the echo pin
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
}

void Altitude_LoadCalibration(float altitudeCalibration)
{
	systematicError = altitudeCalibration;
}

void Altitude_BeginMeasurement(void)
{
    if (inProgress)
    {
        return;
    }

	TCNT0 = 0;
    PORTB |= (1 << TRIG_PIN);
	TCCR0B = (1 << CS00);   // Start timer 0 with no prescaler

    inProgress = true;
}

int Altitude_Get(float *altitude)
{
    if (! dataReady)
    {
        return AAQUAD_BUSY;
    }

    dataReady = false;

    *altitude = measuredAltitude - systematicError;

    return AAQUAD_SUCCEEDED;
}

ISR(TIMER0_COMPA_vect)
{
    PORTB &= ~(1 << TRIG_PIN);
    TCCR0B = 0; // stop timer 0
}

ISR(PCINT0_vect)
{
    static volatile uint16_t previousTimestamp;

    volatile uint16_t thisTimestamp = TCNT3;

    if ((PINB & (1 << PINB0)) == 0)	// falling edge
    {
		volatile int32_t counter;

		if (thisTimestamp < previousTimestamp)  // timer wrapped around
		{
			counter = thisTimestamp + (MAX_16_BIT_VALUE - previousTimestamp);
		}
		else
		{
			counter = thisTimestamp - previousTimestamp;
		}

        volatile float secondsOfTravel = (float) counter * SECONDS_PER_TICK;
        float distance = (secondsOfTravel * SPEED_OF_SOUND) / 2.0f;
        measuredAltitude = distance;
        dataReady = true;
        inProgress = false;
    }

	previousTimestamp = thisTimestamp;
}
