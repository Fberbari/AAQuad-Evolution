#include "unity.h"
#include "PilotInstructions.h"
#include "Common.h"
#include <stdint.h>


#define ACCEPTABLE_ERROR 0.1

#define DUMMY_FLOAT_VALUE 1.0f
#define FLOAT_VALUE_0 14.1f

#define DUMMY_INT_VALUE 20
#define INT_VALUE_0 52
#define INT_VALUE_1 71
#define INT_VALUE_2 14

#define PTR0 0xececfec

#define MAX_16BIT_VALUE 65535U

PilotResult_t DummyPilotResult = {0};

#define DUMMY_VALID_TIMESTAMP_0 DUMMY_INT_VALUE
#define DUMMY_VALID_TIMESTAMP_1  ( DUMMY_VALID_TIMESTAMP_0 + (MIN_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER))


/*****************************************************
* test init and destroy
******************************************************/

void setUp(void)
{
	PilotInstructions_Init();

	// these calls must exists to solely test the Rudder channel wthout worrying about the others
	PilotInstructions_SetXTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetXTimestamp(DUMMY_VALID_TIMESTAMP_1);
	PilotInstructions_SetYTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetYTimestamp(DUMMY_VALID_TIMESTAMP_1);
	PilotInstructions_SetThrottleTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetThrottleTimestamp(DUMMY_VALID_TIMESTAMP_1);
}

void tearDown(void)
{
}

/******************************************************
* tests
******************************************************/

void test_PilotInstructions_HandlesMaxDutyCycleInput(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MAX_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER);

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_FLOAT_WITHIN(ACCEPTABLE_ERROR, 100, ReturnedPilotResult.zPercentage);
}


void test_PilotInstructions_HandlesMinDutyCycleInput(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MIN_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER );

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_FLOAT_WITHIN(ACCEPTABLE_ERROR, -100, ReturnedPilotResult.zPercentage);
}

void test_PilotInstructions_HandlesIntermediateDutyCycleInput(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( ( (MIN_PWM_DUTYCYCLE_S + MAX_PWM_DUTYCYCLE_S) / 2) * F_CPU / TIMER_1_PRESCALER );

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_FLOAT_WITHIN(ACCEPTABLE_ERROR, 0, ReturnedPilotResult.zPercentage);
}

void test_PilotInstructions_ReturnsBusyIfInputTooLarge(void)
{
	// this will often happen when the ISR captures the belly of the waveform instead of the dutycycle

   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + (MAX_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER) + 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfInputTooSmall(void)
{
	// this will infrequently happen if the pcb traces catch some interference ethat causes an Interrupt to fire

   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + (MIN_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER) - 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_HandlesTimerWrapAroundValid(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MAX_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER) - INT_VALUE_0;

	PilotResult_t ReturnedPilotResult;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&ReturnedPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_SUCCEEDED, r);
	TEST_ASSERT_FLOAT_WITHIN(ACCEPTABLE_ERROR, 100, ReturnedPilotResult.zPercentage);
}

void test_PilotInstructions_ReturnsBusyIfTimerWrapAroundAndInputTooLarge(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MAX_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER) - INT_VALUE_0 + 1;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfTimerWrapAroundAndInputTooSmall(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = MAX_16BIT_VALUE - INT_VALUE_0;
	uint16_t timestamp1 = (MIN_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER) - ( INT_VALUE_0 + 1 );

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

void test_PilotInstructions_ReturnsBusyIfUserAttemptsToGetResultBeforeUpdatedDataAvailable(void)
{
   	/***********************SETUP***********************/

	uint16_t timestamp0 = INT_VALUE_0;
	uint16_t timestamp1 = timestamp0 + ( MIN_PWM_DUTYCYCLE_S * F_CPU / TIMER_1_PRESCALER);

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/
	PilotInstructions_SetZTimestamp(timestamp0);
	PilotInstructions_SetZTimestamp(timestamp1);

	PilotInstructions_ComputePilotResult(&DummyPilotResult);

	PilotInstructions_SetXTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetXTimestamp(DUMMY_VALID_TIMESTAMP_1);
	PilotInstructions_SetYTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetYTimestamp(DUMMY_VALID_TIMESTAMP_1);
	PilotInstructions_SetThrottleTimestamp(DUMMY_VALID_TIMESTAMP_0);
	PilotInstructions_SetThrottleTimestamp(DUMMY_VALID_TIMESTAMP_1);

	int r = PilotInstructions_ComputePilotResult(&DummyPilotResult);

	/**********************ASSERTS**********************/

	TEST_ASSERT_EQUAL(AAQUAD_BUSY, r);
}

