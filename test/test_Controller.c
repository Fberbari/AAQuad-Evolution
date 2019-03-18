#include "unity.h"
#include "Controller.h"
#include "mock_PilotInstructions.h"
#include "mock_PwmChip.h"
#include "mock_Pid.h"
#include "mock_SensorData.h"
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

// All first calls of sensor results are with clean structs


PilotResult_t DummyPilotResult = {0};
SensorResults_t DummySensorResults = {0};

/*****************************************************
* test init and destroy
******************************************************/

void setUp(void)
{
	PilotInstructions_Init_Ignore();
	PwmChip_Init_Ignore();
	SensorData_Init_Ignore();
	Pid_Init_Ignore();

	Controller_Init();
}

void tearDown(void)
{
}

/******************************************************
* tests
******************************************************/

void test_Controller_FirstCallOfSensorDataIsWithACleanStruct(void)
{
   	/***********************SETUP***********************/

	SensorResults_t ExpectedSensorResults;
	ExpectedSensorResults.xAngle = 0;
	ExpectedSensorResults.yAngle = 0;
	ExpectedSensorResults.xRate = 0;
	ExpectedSensorResults.yRate = 0;
	ExpectedSensorResults.zRate = 0;
	ExpectedSensorResults.nSamples = 0;

	/********************EXPECTATIONS*******************/

	SensorData_GetResult_ExpectAndReturn(&ExpectedSensorResults, AAQUAD_SUCCEEDED);

	/********************STEPTHROUGH********************/

	Controller_Do();

	/**********************ASSERTS**********************/
}

void test_Controller_AccumulatesSensorDataWhileWaitingForPilotResult(void)
{
   	/***********************SETUP***********************/

	const int NUM_POLLS = 300;

	/********************EXPECTATIONS*******************/

	for(int i = 0; i < NUM_POLLS; i++)
	{
		SensorData_GetResult_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);

		PilotInstructions_ComputePilotResult_ExpectAnyArgsAndReturn(AAQUAD_BUSY);
	}

	/********************STEPTHROUGH********************/

	for(int i = 0; i < (2 * NUM_POLLS); i++)
	{
		Controller_Do();
	}

	/**********************ASSERTS**********************/
}

void test_Controller_CallsPidComputeWithCorrectArgs(void)
{
   	/***********************SETUP***********************/

	float dummyFloatArray[INT_VALUE_0] = {0};

	PilotResult_t PilotResult;
	memset(&PilotResult, INT_VALUE_0, sizeof(PilotResult_t));

	SensorResults_t SensorResults;
	memset(&SensorResults, INT_VALUE_1, sizeof(SensorResults_t));

	/********************EXPECTATIONS*******************/

	SensorData_GetResult_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
		SensorData_GetResult_ReturnThruPtr_SensorResults(&SensorResults);
		
	PilotInstructions_ComputePilotResult_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
		PilotInstructions_ComputePilotResult_ReturnThruPtr_PilotResult(&PilotResult);
	
	Pid_Compute_ExpectAndReturn(&PilotResult, &SensorResults, dummyFloatArray, AAQUAD_SUCCEEDED);
		Pid_Compute_IgnoreArg_motors();

	/********************STEPTHROUGH********************/

	Controller_Do();	// collect Sensor results
	Controller_Do();	// collect pilot result
	Controller_Do();	// compute stage

	/**********************ASSERTS**********************/
}

void test_Controller_CallsPwmSendWithCorrectArgs(void)
{
   	/***********************SETUP***********************/

   	PilotInstructions_ComputePilotResult_IgnoreAndReturn(AAQUAD_SUCCEEDED);
   	SensorData_GetResult_IgnoreAndReturn(AAQUAD_SUCCEEDED);

	float motors[4];
	memset(motors, INT_VALUE_0, (4 * sizeof(float)));

	/********************EXPECTATIONS*******************/
	
	Pid_Compute_ExpectAnyArgsAndReturn(AAQUAD_SUCCEEDED);
		Pid_Compute_ReturnThruPtr_motors(motors);

	PwmChip_Send_ExpectAndReturn(motors, AAQUAD_SUCCEEDED);

	/********************STEPTHROUGH********************/

	Controller_Do();	// collect pilot result
	Controller_Do();	// collect Sensor results
	Controller_Do();	// compute stage
	Controller_Do();	// send to chip

	/**********************ASSERTS**********************/
}

void test_Controller_CallsGetSensorResultWithAClearedStructOnEveryNewLoop(void)
{
   	/***********************SETUP***********************/

    PilotInstructions_ComputePilotResult_IgnoreAndReturn(AAQUAD_SUCCEEDED);
    Pid_Compute_IgnoreAndReturn(AAQUAD_SUCCEEDED);
	PwmChip_Send_IgnoreAndReturn(AAQUAD_SUCCEEDED);

   	SensorResults_t OldSensorResults;
   	OldSensorResults.xAngle = FLOAT_VALUE_0;	// Ensure this one is different than the expected struct


   	SensorResults_t ExpectedSensorResults;
	ExpectedSensorResults.xAngle = 0;
	ExpectedSensorResults.yAngle = 0;
	ExpectedSensorResults.xRate = 0;
	ExpectedSensorResults.yRate = 0;
	ExpectedSensorResults.zRate = 0;
	ExpectedSensorResults.nSamples = 0;

	const int NUM_LOOPS = 300;

	/********************EXPECTATIONS*******************/
	
	for (int i = 0; i < NUM_LOOPS; i++)
	{
		SensorData_GetResult_ExpectAndReturn(&ExpectedSensorResults, AAQUAD_SUCCEEDED);
			SensorData_GetResult_ReturnThruPtr_SensorResults(&OldSensorResults);
	}


	/********************STEPTHROUGH********************/

	for (int i = 0; i < NUM_LOOPS; i++)
	{
		Controller_Do();	// collect Sensor results
		Controller_Do();	// collect pilot result
		Controller_Do();	// Process results
		Controller_Do();	// send to chip
		Controller_Do();	// reset to prepare for next loop
	}

	/**********************ASSERTS**********************/
}
