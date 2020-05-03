#include "Controller.h"
#include "PilotInstructions.h"
#include "Imu.h"
#include "Altitude.h"
#include "MahonyAHRS.h"
#include "Pid.h"
#include "PwmChip.h"
#include "Leds.h"

#include "Common.h"

#include <avr/interrupt.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

typedef enum state
{
	CTRL_GET_PILOT_INSTRUCTIONS,
	CTRL_GET_IMU_DATA,
    CTRL_GET_ALTITUDE,
    CTRL_TRIGGER_NEW_MEASUREMENTS,
    CTRL_FUSE_MEASUREMENTS,
    CTRL_CONVERT_TO_EULER,
	CTRL_COMPUTE_PID,
	CTRL_SEND_TO_PWM,
	CTRL_WAIT_FOR_TIMER,
	CTRL_FAILED

}Controller_State_t;

#define TIMER_VAL_1_PERIOD ((uint16_t) ((float) CTRL_LOOP_PERIOD / (8.0f / (float)F_CPU)))   // assuming clk divider of 8.

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static Controller_State_t currentState;

static PilotResult_t PilotResult;
static ImuData_t ImuData;
static float altitude;
static EulerXYZ_t EulerAngles;
static EulerRates_t EulerRates;
static float motors[4];

static bool periodHasPassed;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static Controller_State_t GetPilotInstructions_State(void);
static Controller_State_t GetImuData_State(void);
static Controller_State_t GetAltitude_State(void);
static Controller_State_t TriggerNewMeasurements_State(void);
static Controller_State_t FuseMeasurements_State(void);
static Controller_State_t ConvertToEuler_State(void);
static Controller_State_t ComputePidState(void);
static Controller_State_t SendToPwm_State(void);
static Controller_State_t WaitForTimer_State(void);

static void InitData(void);
static void InitPeriodTimer(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Controller_Init(void)
{
    sei();  // global interrupt enable
	PilotInstructions_Init();
    Imu_Init();
    Altitude_Init();
    Pid_Init();
	PwmChip_Init();
    Leds_Init();

    InitData();
    InitPeriodTimer();

	currentState = CTRL_TRIGGER_NEW_MEASUREMENTS;
}

void Controller_DoYourThingAndFlyTheQuadITrustYou(void)
{
	Controller_State_t nextState;
	switch(currentState)
	{
		case CTRL_GET_PILOT_INSTRUCTIONS:
			nextState = GetPilotInstructions_State();
			break;

		case CTRL_GET_IMU_DATA:
			nextState = GetImuData_State();
			break;

        case CTRL_GET_ALTITUDE:
            nextState = GetAltitude_State();
            break;

        case CTRL_TRIGGER_NEW_MEASUREMENTS:
            nextState = TriggerNewMeasurements_State();
            break;

        case CTRL_FUSE_MEASUREMENTS:
            nextState = FuseMeasurements_State();
            break;

        case CTRL_CONVERT_TO_EULER:
            nextState = ConvertToEuler_State();
            break;

		case CTRL_SEND_TO_PWM:
			nextState = SendToPwm_State();
			break;

        case CTRL_COMPUTE_PID:
            nextState = ComputePidState();
            break;

        case CTRL_WAIT_FOR_TIMER:
            nextState = WaitForTimer_State();
            break;

		default:
			nextState = CTRL_FAILED;
			break;
	}

	currentState = nextState;
}

static void InitData(void)
{
    // All this initial data represents a quadcopter that's on
    // level ground, not moving and pointing in some arbitrary direction.
    PilotResult.xPercentage = 0.0f;
    PilotResult.yPercentage = 0.0f;
    PilotResult.zPercentage = 0.0f;
    PilotResult.throttlePercentage = 0.0f;

    ImuData.gyrX = 0.0f;
    ImuData.gyrY = 0.0f;
    ImuData.gyrZ = 0.0f;
    ImuData.accX = 0.0f;
    ImuData.accY = 0.0f;
    ImuData.accZ = 1.0f;
    ImuData.magX = 1.0f;
    ImuData.magY = 0.0f;
    ImuData.magZ = 0.0f;

    altitude = 0.0f;
}

static void InitPeriodTimer(void)
{
    TIMSK4 = (1 << OCIE4A); // interrupt on match
    OCR4A = TIMER_VAL_1_PERIOD;
    TCCR4B = (1 << CS41); // clk divided by 8
}

static Controller_State_t GetPilotInstructions_State(void)
{
	int r = PilotInstructions_ComputePilotResult(&PilotResult);

    if(r == AAQUAD_FAILED)
	{
		return CTRL_FAILED;
	}

	return CTRL_GET_IMU_DATA;
}

static Controller_State_t GetImuData_State(void)
{

	int r = Imu_GetResult(&ImuData);

    if(r == AAQUAD_FAILED)
    {
        return CTRL_FAILED;
    }

    return CTRL_GET_ALTITUDE;
}

static Controller_State_t GetAltitude_State(void)
{

    int r = Altitude_Get(&altitude);

    if(r == AAQUAD_FAILED)
    {
        return CTRL_FAILED;
    }

    return CTRL_TRIGGER_NEW_MEASUREMENTS;
}

static Controller_State_t TriggerNewMeasurements_State(void)
{
    Altitude_BeginMeasurement();
    Imu_BeginRead();

    return CTRL_FUSE_MEASUREMENTS;
}

static Controller_State_t FuseMeasurements_State(void)
{
    // this module makes the return data available as the global variables q0, q1, q2 and q3
    MahonyAHRSupdate(ImuData.gyrX, ImuData.gyrY, ImuData.gyrZ, ImuData.accX, ImuData.accY, ImuData.accZ, ImuData.magX, ImuData.magY, ImuData.magZ);

    return CTRL_CONVERT_TO_EULER;
}

static Controller_State_t ConvertToEuler_State(void)
{
    quat2Euler(q0, q1, q2, q3, &EulerAngles);

    gyro2EulerRates(&EulerAngles, ImuData.gyrX, ImuData.gyrY, ImuData.gyrZ, &EulerRates);

    return CTRL_COMPUTE_PID;
}

static Controller_State_t ComputePidState(void)
{

    int r = Pid_Compute(&PilotResult, &EulerAngles, &EulerRates, altitude, motors);

    if(r == AAQUAD_FAILED)
    {
        return CTRL_FAILED;
    }

    return CTRL_SEND_TO_PWM;
}

static Controller_State_t SendToPwm_State(void)
{
	int r = PwmChip_Send(motors);

    if(r == AAQUAD_FAILED)
    {
        return CTRL_FAILED;
    }

	return CTRL_WAIT_FOR_TIMER;
}

static Controller_State_t WaitForTimer_State(void)
{
    if (periodHasPassed)
    {
        periodHasPassed = false;

        return CTRL_GET_PILOT_INSTRUCTIONS;
    }

    return CTRL_WAIT_FOR_TIMER;
}

ISR(TIMER4_COMPA_vect)
{
    TCNT4 = 0;
    periodHasPassed = true;
}
