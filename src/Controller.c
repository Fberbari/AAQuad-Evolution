#include "Controller.h"
#include "PilotInstructions.h"
#include "PwmChip.h"
#include "Pid.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

typedef enum state
{
	CTRL_GET_PILOT_RESULT,
	CTRL_COMPUTE_RESULT,
	CTRL_SEND_TO_PWM,
	CTRL_FAILED

}Controller_State_t;

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static Controller_State_t currentState;

static PilotResult_t PilotResult;
static float motors[4];

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static Controller_State_t GetPilotResult_State(void);
static Controller_State_t ComputeResult_State(void);
static Controller_State_t SendToPwm_State(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Controller_Init(void)
{
	PilotInstructions_Init();
	PwmChip_Init();

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

		case CTRL_COMPUTE_RESULT:
			nextState = ComputeResult_State();
			break;

		case CTRL_SEND_TO_PWM:
			nextState = SendToPwm_State();
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
		return CTRL_COMPUTE_RESULT;
	}

	return CTRL_FAILED;
}

static Controller_State_t ComputeResult_State(void)
{
	Pid_Compute(&PilotResult, motors);

	return CTRL_SEND_TO_PWM;
}

static Controller_State_t SendToPwm_State(void)
{
	PwmChip_Send(motors);

	return CTRL_GET_PILOT_RESULT;
}