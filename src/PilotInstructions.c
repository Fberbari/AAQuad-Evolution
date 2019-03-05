#include "PilotInstructions.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define MAX_16_BIT_VALUE 65535U


typedef struct 
{
	float throttle;	// dutyCycle of the throttle channel in seconds

}DutyCycle_t;

typedef struct
{
	bool throttle;

}NewDataAvailable_t;

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

NewDataAvailable_t NewDataAvailable;

static uint16_t newestThrottleTimestamp;
static uint16_t previousThrottleTimestamp; 


/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void GetDutyCycles(DutyCycle_t *DutyCycle);

static void SetAllDataOld(void);

static float ComputeThrottlePower(float throttleDutyCycle);

static bool ThrottleTimerWrappedAround(void);

static bool CapturedWaveformsAreValid(DutyCycle_t *DutyCycle);
static bool AllDataIsNew(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void PilotInstructions_Init(void)
{
	NewDataAvailable.throttle = false;
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

	PilotResult->throttlePower = ComputeThrottlePower(DutyCycle.throttle);

	SetAllDataOld();

	return AAQUAD_SUCCEEDED;
}

void PilotInstructions_SetThrottleTimestamp(uint16_t throttleTimestamp)
{
	previousThrottleTimestamp = newestThrottleTimestamp;
	newestThrottleTimestamp = throttleTimestamp;

	NewDataAvailable.throttle = true;
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
}

static bool CapturedWaveformsAreValid(DutyCycle_t *DutyCycle)
{
	if ( (DutyCycle->throttle < MIN_PWM_DUTYCYCLE_S) || (DutyCycle->throttle > MAX_PWM_DUTYCYCLE_S) )
	{
		return false;
	}

	return true;
}

static float ComputeThrottlePower(float throttleDutyCycle)
{
	return map(throttleDutyCycle, MIN_PWM_DUTYCYCLE_S, MAX_PWM_DUTYCYCLE_S, 0, 100);
}

static bool ThrottleTimerWrappedAround(void)
{
	if (newestThrottleTimestamp < previousThrottleTimestamp)
	{
		return true;
	}

	return false;
}

static bool AllDataIsNew(void)
{
	if(NewDataAvailable.throttle)
	{
		return true;
	}

	return false;
}

static void SetAllDataOld(void)
{
	NewDataAvailable.throttle = false;
}
