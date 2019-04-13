#include "unity.h"
#include "LowPassFilter.h"

// rejects negative or 0 window size

/*****************************************************
* test init and destroy
******************************************************/

void setUp(void)
{
	LowPassFilter_Init();
}

void tearDown(void)
{
}

/******************************************************
* tests
******************************************************/


void test_LowPassFilter_Rejects0WindowSize(void)
{
   	/***********************SETUP***********************/
	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	LowPassFilter_t Filter = LowPassFilter_CreateFilter(0);

	/**********************ASSERTS**********************/

	TEST_ASSERT_NULL(Filter);
}

// ceedling by itself cannot test this. This should be run under valgrind.
void test_LowPassFilter_memoryIsProperlyDeallocated(void)
{
   	/***********************SETUP***********************/

	unsigned int validWindowSize = 5;

	/********************EXPECTATIONS*******************/
	/********************STEPTHROUGH********************/

	LowPassFilter_t Filter = LowPassFilter_CreateFilter(validWindowSize);
	LowPassFilter_DestroyFilter(Filter);

	/**********************ASSERTS**********************/
}
