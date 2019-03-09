#include "unity.h"
#include "Controller.h"
#include "mock_PilotInstructions.h"
#include "mock_PwmChip.h"
#include "mock_Pid.h"
#include "Common.h"
#include <string.h>

#define DUMMY_FLOAT_VALUE 1.0f
#define FLOAT_VALUE_0 14.1f

#define DUMMY_INT_VALUE 20
#define INT_VALUE_0 52
#define INT_VALUE_1 71
#define INT_VALUE_2 14

#define PTR0 0xececfec

#define MAX_16BIT_VALUE 65535U

PilotResult_t DummyPilotResult = {0};

/*****************************************************
* test init and destroy
******************************************************/

void setUp(void)
{
	PilotInstructions_Init_Ignore();
	PwmChip_Init_Ignore();

	Controller_Init();
}

void tearDown(void)
{
}

/******************************************************
* tests
******************************************************/

void test_Controller_CallsComputePilotResultWhileBusy(void)
{
   	/***********************SETUP***********************/

	const int NUM_POLLS = 300;

	/********************EXPECTATIONS*******************/

	for(int i = 0; i < NUM_POLLS; i++)
	{
		PilotInstructions_ComputePilotResult_ExpectAndReturn(&DummyPilotResult, AAQUAD_BUSY);
			PilotInstructions_ComputePilotResult_IgnoreArg_PilotResult();
	}

	/********************STEPTHROUGH********************/

	for(int i = 0; i < NUM_POLLS; i++)
	{
		Controller_Do();
	}

	/**********************ASSERTS**********************/
}

void test_Controller_CallsPidComputeWithCorrectArgsOncePilotResultGot(void)
{
   	/***********************SETUP***********************/

	float dummyFloatArray[INT_VALUE_0] = {0};

	PilotResult_t PilotResult;
	memset(&PilotResult, INT_VALUE_0, sizeof(PilotResult_t));

	/********************EXPECTATIONS*******************/
		
	PilotInstructions_ComputePilotResult_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
		PilotInstructions_ComputePilotResult_ReturnThruPtr_PilotResult(&PilotResult);
	
	Pid_Compute_ExpectAndReturn(&PilotResult, dummyFloatArray, AAQUAD_SUCCEEDED);
		Pid_Compute_IgnoreArg_motors();

	/********************STEPTHROUGH********************/

	Controller_Do();
	Controller_Do();

	/**********************ASSERTS**********************/
}

void test_Controller_CallsPwmSendWithCorrectArgsOncePilotResultGot(void)
{
   	/***********************SETUP***********************/

   	PilotInstructions_ComputePilotResult_IgnoreAndReturn(AAQUAD_SUCCEEDED);

	float motors[4];
	memset(motors, INT_VALUE_0, (4 * sizeof(float)));

	/********************EXPECTATIONS*******************/
	
	Pid_Compute_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
		Pid_Compute_ReturnThruPtr_motors(motors);

	PwmChip_Send_ExpectAndReturn(motors, AAQUAD_SUCCEEDED);

	/********************STEPTHROUGH********************/

	Controller_Do();
	Controller_Do();
	Controller_Do();

	/**********************ASSERTS**********************/
}

void test_Controller_RepeatsCycle(void)
{
   	/***********************SETUP***********************/

	Pid_Compute_IgnoreAndReturn(AAQUAD_SUCCEEDED);
	
	float motors[4];
	memset(motors, INT_VALUE_0, (4 * sizeof(float)));

	/********************EXPECTATIONS*******************/
	PilotInstructions_ComputePilotResult_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
	PwmChip_Send_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);

	PilotInstructions_ComputePilotResult_ExpectAndReturn(&DummyPilotResult, AAQUAD_BUSY);
		PilotInstructions_ComputePilotResult_IgnoreArg_PilotResult();

	/********************STEPTHROUGH********************/

	Controller_Do();
	Controller_Do();
	Controller_Do();
	Controller_Do();

	/**********************ASSERTS**********************/
}