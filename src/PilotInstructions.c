#include "PilotInstructions.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define MAX_16_BIT_VALUE 65535U

#define TIMER_1_PRESCALER	8U

#define MAX_PWM_DUTYCYCLE_S 0.002f
#define MIN_PWM_DUTYCYCLE_S	0.001f

typedef struct
{
	float throttle;	// real-time dutyCycle of the 4 channels in seconds
	float x;
	float y;
	float z;

}DutyCycle_t;

typedef struct
{
	bool throttle;
	bool x;
	bool y;
	bool z;

}NewDataAvailable_t;

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

NewDataAvailable_t NewDataAvailable;

static volatile uint16_t newestThrottleTimestamp;
static volatile uint16_t previousThrottleTimestamp;
static volatile uint16_t newestXTimestamp;
static volatile uint16_t previousXTimestamp;
static volatile uint16_t newestYTimestamp;
static volatile uint16_t previousYTimestamp;
static volatile uint16_t newestZTimestamp;
static volatile uint16_t previousZTimestamp;

static float xZeroOffset;
static float yZeroOffset;	// TODO I'd like to see this be a struct
static float zZeroOffset;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void InitInterrupts(void);

static void GetDutyCycles(DutyCycle_t *DutyCycle);

static void SetAllDataOld(void);

static float ComputeThrottlePercentage(float throttleDutyCycle);
static float ComputeXPercentage(float XDutyCycle);
static float ComputeYPercentage(float YDutyCycle);
static float ComputeZPercentage(float ZDutyCycle);

static bool ThrottleTimerWrappedAround(void);
static bool XTimerWrappedAround(void);
static bool YTimerWrappedAround(void);
static bool ZTimerWrappedAround(void);

static bool CapturedWaveformsAreValid(DutyCycle_t *DutyCycle);
static bool AllDataIsNew(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void PilotInstructions_Init(void)
{
	NewDataAvailable.throttle = false;
	NewDataAvailable.x = false;
	NewDataAvailable.y = false;
	NewDataAvailable.z = false;

	xZeroOffset = 0.0f;
	yZeroOffset = 0.0f;
	zZeroOffset = 0.0f;

	InitInterrupts();
}

int PilotInstructions_ComputePilotResult(PilotResult_t *PilotResult)
{
	if ( ! AllDataIsNew() )
	{
		return AAQUAD_BUSY;
	}

	DutyCycle_t DutyCycle;
	GetDutyCycles(&DutyCycle);

	if ( ! CapturedWaveformsAreValid(&DutyCycle) )
	{
		return AAQUAD_BUSY;
	}

	PilotResult->throttlePercentage = ComputeThrottlePercentage(DutyCycle.throttle);
	PilotResult->xPercentage = ComputeXPercentage(DutyCycle.x) - xZeroOffset;
	PilotResult->yPercentage = ComputeYPercentage(DutyCycle.y) - yZeroOffset;
	PilotResult->zPercentage = ComputeZPercentage(DutyCycle.z) - zZeroOffset;

	SetAllDataOld();

	return AAQUAD_SUCCEEDED;
}

static void InitInterrupts(void)
{
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
}

static void GetDutyCycles(DutyCycle_t *DutyCycle)
{
	if (ThrottleTimerWrappedAround())
	{
		DutyCycle->throttle = ( ( (float) (newestThrottleTimestamp + (MAX_16_BIT_VALUE - previousThrottleTimestamp)) * TIMER_1_PRESCALER ) / F_CPU);
	}
	else
	{
		DutyCycle->throttle = ( ( (float) (newestThrottleTimestamp - previousThrottleTimestamp)  * TIMER_1_PRESCALER ) / F_CPU);
	}

	if (XTimerWrappedAround())
	{
		DutyCycle->x = ( ( (float) (newestXTimestamp + (MAX_16_BIT_VALUE - previousXTimestamp)) * TIMER_1_PRESCALER ) / F_CPU);
	}
	else
	{
		DutyCycle->x = ( ( (float) (newestXTimestamp - previousXTimestamp)  * TIMER_1_PRESCALER ) / F_CPU);
	}

	if (YTimerWrappedAround())
	{
		DutyCycle->y = ( ( (float) (newestYTimestamp + (MAX_16_BIT_VALUE - previousYTimestamp)) * TIMER_1_PRESCALER ) / F_CPU);
	}
	else
	{
		DutyCycle->y = ( ( (float) (newestYTimestamp - previousYTimestamp)  * TIMER_1_PRESCALER ) / F_CPU);
	}

	if (ZTimerWrappedAround())
	{
		DutyCycle->z = ( ( (float) (newestZTimestamp + (MAX_16_BIT_VALUE - previousZTimestamp)) * TIMER_1_PRESCALER ) / F_CPU);
	}
	else
	{
		DutyCycle->z = ( ( (float) (newestZTimestamp - previousZTimestamp)  * TIMER_1_PRESCALER ) / F_CPU);
	}
}

static bool CapturedWaveformsAreValid(DutyCycle_t *DutyCycle)
{
	if ( (DutyCycle->throttle < MIN_PWM_DUTYCYCLE_S) || (DutyCycle->throttle > MAX_PWM_DUTYCYCLE_S) )
	{
		return false;
	}

	if ( (DutyCycle->x < MIN_PWM_DUTYCYCLE_S) || (DutyCycle->x > MAX_PWM_DUTYCYCLE_S) )
	{
		return false;
	}

	if ( (DutyCycle->y < MIN_PWM_DUTYCYCLE_S) || (DutyCycle->y > MAX_PWM_DUTYCYCLE_S) )
	{
		return false;
	}

	if ( (DutyCycle->z < MIN_PWM_DUTYCYCLE_S) || (DutyCycle->z > MAX_PWM_DUTYCYCLE_S) )
	{
		return false;
	}

	return true;
}

static float ComputeThrottlePercentage(float throttleDutyCycle)
{
	return map(throttleDutyCycle, MIN_PWM_DUTYCYCLE_S, MAX_PWM_DUTYCYCLE_S, 0, 100);
}

static float ComputeXPercentage(float xDutyCycle)
{
	return map(xDutyCycle, MIN_PWM_DUTYCYCLE_S, MAX_PWM_DUTYCYCLE_S, -100, 100);
}

static float ComputeYPercentage(float yDutyCycle)
{
	return map(yDutyCycle, MIN_PWM_DUTYCYCLE_S, MAX_PWM_DUTYCYCLE_S, -100, 100);
}

static float ComputeZPercentage(float zDutyCycle)
{
	return map(zDutyCycle, MIN_PWM_DUTYCYCLE_S, MAX_PWM_DUTYCYCLE_S, -100, 100);
}

static bool ThrottleTimerWrappedAround(void)
{
	if (newestThrottleTimestamp < previousThrottleTimestamp)
	{
		return true;
	}

	return false;
}

static bool XTimerWrappedAround(void)
{
	if (newestXTimestamp < previousXTimestamp)
	{
		return true;
	}

	return false;
}

static bool YTimerWrappedAround(void)
{
	if (newestYTimestamp < previousYTimestamp)
	{
		return true;
	}

	return false;
}

static bool ZTimerWrappedAround(void)
{
	if (newestZTimestamp < previousZTimestamp)
	{
		return true;
	}

	return false;
}

static bool AllDataIsNew(void)
{
	if (NewDataAvailable.throttle && NewDataAvailable.x && NewDataAvailable.y && NewDataAvailable.z)
	{
		return true;
	}

	return false;
}

static void SetAllDataOld(void)
{
	NewDataAvailable.throttle = false;
	NewDataAvailable.x = false;
	NewDataAvailable.y = false;
	NewDataAvailable.z = false;
}

ISR(INT0_vect)		// aileron
{
	previousYTimestamp = newestYTimestamp;
	newestYTimestamp = TCNT1;
	NewDataAvailable.y = true;
}

ISR(INT1_vect)		// throttle
{

	previousThrottleTimestamp = newestThrottleTimestamp;
	newestThrottleTimestamp = TCNT1;
	NewDataAvailable.throttle = true;
}

ISR(PCINT1_vect)	// rudder
{
	previousZTimestamp = newestZTimestamp;
	newestZTimestamp = TCNT1;
	NewDataAvailable.z = true;
}

ISR(PCINT2_vect)	// elevator
{
	previousXTimestamp = newestXTimestamp;
	newestXTimestamp = TCNT1;
	NewDataAvailable.x = true;
}
