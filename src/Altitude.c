#include "Altitude.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define SPEED_OF_SOUND 340  // m/s

#define SECONDS_PER_TICK (1 / F_CPU)    // assuming timer has no presaler

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

/**********************************************************
 * Code
 *********************************************************/

void Altitude_Init(void)
{
    // timer 0 used to manage the trigger pulse
    // interrupts are iniatialised but the counter is not started yet
	TIMSK0 = (1 << OCIE0A);
    OCR0A = TIMER_VAL_11_US;

    // timer 2 used to measure the length of the echo pulse
    TCCR2B = (1 << CS20);

    DDRB |= (1 << TRIG_PIN);
    PORTB &= ~(1 << TRIG_PIN);

    // pcint for the echo pin
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
}

void Altitude_BeginMeasurement(void)
{
	TCNT0 = 0;
    PORTB |= (1 << TRIG_PIN);
	TCCR0B = (1 << CS00);   // Start timer 0 with no prescaler
}

int Altitude_Get(float *altitude)
{
    if (! dataReady)
    {
        return AAQUAD_BUSY;
    }

    dataReady = false;

    *altitude = measuredAltitude;

    return AAQUAD_SUCCEEDED;
}

ISR(TIMER0_COMPA_vect)
{
    PORTB &= ~(1 << TRIG_PIN);
    TCCR0B = 0; // stop timer 0
}

ISR(PCINT0_vect)
{
    static uint16_t previousTimestamp;
    static bool isFallingEdge;

    uint16_t thisTimestamp = TCNT2;

    int32_t counter = thisTimestamp - previousTimestamp;

    previousTimestamp = thisTimestamp;

    if (isFallingEdge == true)
    {
        volatile float secondsOfTravel = fabs( counter * SECONDS_PER_TICK);
        float distance = (secondsOfTravel * SPEED_OF_SOUND) / 2;
        measuredAltitude = distance;
        dataReady = true;
    }

    isFallingEdge ^= 1;
}
