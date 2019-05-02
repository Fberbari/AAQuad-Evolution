#include "PilotInstructions.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define MAX_16_BIT_VALUE 65535U


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

static uint16_t newestThrottleTimestamp;
static uint16_t previousThrottleTimestamp; 
static uint16_t newestXTimestamp;
static uint16_t previousXTimestamp; 
static uint16_t newestYTimestamp;
static uint16_t previousYTimestamp; 
static uint16_t newestZTimestamp;
static uint16_t previousZTimestamp; 

static float xZeroOffset;
static float yZeroOffset;
static float zZeroOffset;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

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
}

void PilotInstructions_Calibrate(void)
{
	PilotResult_t CalibrationZeros;

	volatile int nRunsForReliableAverage = 50;

	float xOffset = 0;
	float yOffset = 0;
	float zOffset = 0;


	for (volatile int i = 0; i < nRunsForReliableAverage; i++)
	{
		while(PilotInstructions_ComputePilotResult(&CalibrationZeros) != AAQUAD_SUCCEEDED)
		{
			asm("nop");
		}

		xOffset += CalibrationZeros.xPercentage;
		yOffset += CalibrationZeros.yPercentage;
		zOffset += CalibrationZeros.zPercentage;
	}

	xOffset /= (float) nRunsForReliableAverage;
	yOffset /= (float) nRunsForReliableAverage;
	zOffset /= (float) nRunsForReliableAverage;

	xZeroOffset = xOffset;
	yZeroOffset = yOffset;
	zZeroOffset = zOffset;
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

void PilotInstructions_SetThrottleTimestamp(uint16_t throttleTimestamp)
{
	previousThrottleTimestamp = newestThrottleTimestamp;
	newestThrottleTimestamp = throttleTimestamp;

	NewDataAvailable.throttle = true;
}

void PilotInstructions_SetXTimestamp(uint16_t xTimestamp)
{
	previousXTimestamp = newestXTimestamp;
	newestXTimestamp = xTimestamp;

	NewDataAvailable.x = true;
}

void PilotInstructions_SetYTimestamp(uint16_t yTimestamp)
{
	previousYTimestamp = newestYTimestamp;
	newestYTimestamp = yTimestamp;

	NewDataAvailable.y = true;
}

void PilotInstructions_SetZTimestamp(uint16_t zTimestamp)
{
	previousZTimestamp = newestZTimestamp;
	newestZTimestamp = zTimestamp;

	NewDataAvailable.z = true;
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
	if( ! NewDataAvailable.throttle)
	{
		return false;
	}

	if( ! NewDataAvailable.x)
	{
		return false;
	}

	if( ! NewDataAvailable.y)
	{
		return false;
	}

	if( ! NewDataAvailable.z)
	{
		return false;
	}

	return true;
}

static void SetAllDataOld(void)
{
	NewDataAvailable.throttle = false;
	NewDataAvailable.x = false;
	NewDataAvailable.y = false;
	NewDataAvailable.z = false;
}
