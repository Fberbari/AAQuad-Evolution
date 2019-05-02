#include "Controller.h"
#include "PilotInstructions.h"
#include "SensorData.h"
#include "PwmChip.h"
#include "Pid.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

typedef enum state
{
	CTRL_GET_PILOT_RESULT,
	CTRL_GET_SENSOR_DATA,
	CTRL_PROCESS_RESULTS,
	CTRL_SEND_TO_PWM,
	CTRL_RESET_FOR_NEXT_LOOP,
	CTRL_FAILED

}Controller_State_t;


/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static Controller_State_t currentState;

static PilotResult_t PilotResult;
static SensorResults_t SensorResults;

static float motors[4];

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static Controller_State_t GetPilotResult_State(void);
static Controller_State_t GetSensorData_State(void);
static Controller_State_t ProcessResults_State(void);
static Controller_State_t SendToPwm_State(void);
static Controller_State_t ResetForNextLoop_State(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Controller_Init(void)
{
	PilotInstructions_Init();
	PwmChip_Init();
	SensorData_Init();
	Pid_Init();
	
	DDRB |= ((1 << 1) | (1 << 0));	// TODO make a module for the led's

	SensorResults.xAccAngle = 0;
	SensorResults.yAccAngle = 0;
	SensorResults.xGyroRate = 0;
	SensorResults.yGyroRate = 0;
	SensorResults.zGyroRate = 0;
	SensorResults.nSamples = 0;

	currentState = CTRL_GET_PILOT_RESULT;

}

void Controller_Do(void)
{
	Controller_State_t nextState;
	switch(currentState)
	{
		case CTRL_GET_PILOT_RESULT:
			nextState = GetPilotResult_State();
			break;
			
		case CTRL_GET_SENSOR_DATA:
			nextState = GetSensorData_State();
			break;

		case CTRL_PROCESS_RESULTS:
			nextState = ProcessResults_State();
			break;

		case CTRL_SEND_TO_PWM:
			nextState = SendToPwm_State();
			break;

		case CTRL_RESET_FOR_NEXT_LOOP:
			nextState = ResetForNextLoop_State();
			break;

		default:
			nextState = CTRL_FAILED;
			break;
	}

	currentState = nextState;
}

static Controller_State_t GetPilotResult_State(void)
{
	int r = PilotInstructions_ComputePilotResult(&PilotResult);
	if(r == AAQUAD_BUSY)
	{
		return CTRL_GET_PILOT_RESULT;
	}

	if(r == AAQUAD_SUCCEEDED)
	{
		return CTRL_GET_SENSOR_DATA;
	}

	return CTRL_FAILED;
}

static Controller_State_t GetSensorData_State(void)
{
	
	SensorData_GetResult(&SensorResults);

	return CTRL_PROCESS_RESULTS;
}

static Controller_State_t ProcessResults_State(void)
{
	Pid_Compute(&PilotResult, &SensorResults, motors);

	return CTRL_SEND_TO_PWM;
}

static Controller_State_t SendToPwm_State(void)
{
	PwmChip_Send(motors);

	return CTRL_RESET_FOR_NEXT_LOOP;
}

static Controller_State_t ResetForNextLoop_State(void)
{
	SensorResults.xAccAngle = 0;
	SensorResults.yAccAngle = 0;
	SensorResults.xGyroRate = 0;
	SensorResults.yGyroRate = 0;
	SensorResults.zGyroRate = 0;
	SensorResults.nSamples = 0;

	PORTB ^= (1 << 0);		// indicate control loop has occured

	return CTRL_GET_PILOT_RESULT;
}