#include "unity.h"
#include "PilotInstructions.h"
#include "Common.h"
#include <stdint.h>


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
	PilotInstructions_Init();
}

void tearDown(void)
{
}

/******************************************************
* tests
******************************************************/

// NOTE: In this first stage ofdevelopement, only the throttle channel is tested


void test_PilotInstructions_HandlesMaxDutyCycleInput(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MAX_PWM_DUTYCYCLE_S * F_CPU );

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_EQUAL_FLOAT(100, ReturnedPilotResult.throttlePower);
}

void test_PilotInstructions_HandlesMinDutyCycleInput(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MIN_PWM_DUTYCYCLE_S * F_CPU );

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_EQUAL_FLOAT(0, ReturnedPilotResult.throttlePower);
}

void test_PilotInstructions_ReturnsBusyIfInputTooLarge(void)
{
	// this will often happen when the ISR captures the belly of the waveform instead f the dutycycle

   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + (MAX_PWM_DUTYCYCLE_S * F_CPU) + 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfInputTooSmall(void)
{
	// this will infrequently happen if the pcb traces catch some interference ethat causes an Interrupt to fire

   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + (MIN_PWM_DUTYCYCLE_S * F_CPU) - 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_HandlesTimerWrapAroundValid(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MAX_PWM_DUTYCYCLE_S * F_CPU ) - INT_VALUE_0;

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_EQUAL_FLOAT(100, ReturnedPilotResult.throttlePower);
}

void test_PilotInstructions_ReturnsBusyIfTimerWrapAroundAndInputTooLarge(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MAX_PWM_DUTYCYCLE_S * F_CPU ) - INT_VALUE_0 + 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfTimerWrapAroundAndInputTooSmall(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MIN_PWM_DUTYCYCLE_S * F_CPU ) - ( INT_VALUE_0 + 1 );

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfUserAttemptsToGetResultBeforeUpdatedDataAvailable(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MIN_PWM_DUTYCYCLE_S * F_CPU );

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetThrottleTimestamp(timestamp0);
	PilotInstructions_SetThrottleTimestamp(timestamp1);

	PilotInstructions_ComputePilotResult(&DummyPilotResult);
	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}