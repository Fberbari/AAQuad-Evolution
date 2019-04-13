
#include "testCases.h"
#include "../src/Common.h"
#include "../src/LowPassFilter.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define CURRENT_TEST		flatTest		// this should be the name of the array in the .h file
#define OUTPUT_FILE_NAME 	"flatTest.csv"

#define SIZE_OF_SIMULATION	150
#define NUM_WINDOWS	4

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static FILE *fPointer;

static LowPassFilter_t Filters[NUM_WINDOWS];
static float results[NUM_WINDOWS];

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void Init(void);

static void OutputToFile(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
	Init();

	for (int i = 0; i < SIZE_OF_SIMULATION; i++)
	{

		for (int j = 0; j < NUM_WINDOWS; j++)
		{
			results[j] = LowPassFilter_Execute(Filters[j], CURRENT_TEST[i]);
		}

		OutputToFile();
	}

	fclose(fPointer);
	
	return 0;
}


static void Init(void)
{
	LowPassFilter_Init();

	for(int i = 0; i < NUM_WINDOWS; i++)
	{
		Filters[i] = LowPassFilter_CreateFilter((5 * i) + 5);
	}

	fPointer = fopen("SimulationResults/" OUTPUT_FILE_NAME, "w");
	fclose(fPointer);
	fPointer = fopen("SimulationResults/" OUTPUT_FILE_NAME, "a");
	fprintf(fPointer, ", Unfiltered Data, Various Window Sizes");
}

static void OutputToFile()
{
	fprintf(fPointer, "\n");

	static int i;

	fprintf(fPointer, "%d %s", i, ",");

	fprintf(fPointer, "%f %s", CURRENT_TEST[i], ",");

	i++;

	for (int j = 0; j < NUM_WINDOWS; j++)
	{
		fprintf(fPointer, "%f %s", results[j], ",");
	}

}