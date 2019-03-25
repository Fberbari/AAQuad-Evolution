
#include "testCases.h"
#include "../src/Common.h"
#include "../src/Pid.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define CURRENT_TEST		stepTest		// this should be the name of the array in the .h file
#define OUTPUT_FILE_NAME 	"stepTest.csv"

#define RADIANS_TO_DEGREES	57.3

#define SIZE_OF_SIMULATION	250				// 250 pilot instruction equates to 5s
#define THROTTLE_POWER		50

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static FILE *fPointer;

static float motorPercentages[4];
static SensorResults_t QuadOrientation;

static PilotResult_t PilotInstruction;


/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void Init(void);

static void SimulateQuadPosition(float *motorPercentages, SensorResults_t *QuadOrientation);

static void GetNextPilotInstruction(PilotResult_t *PilotInstruction);

static void OutputToFile(PilotResult_t *PilotInstruction, SensorResults_t *QuadOrientation, float *motorPercentages);

static float GetAngularAcceleration(float *motorPercentages);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
	Init();

	for (int i = 0; i < SIZE_OF_SIMULATION; i++)
	{

		GetNextPilotInstruction(&PilotInstruction);

		Pid_Compute(&PilotInstruction, &QuadOrientation, motorPercentages);

		SimulateQuadPosition(motorPercentages, &QuadOrientation);

		OutputToFile(&PilotInstruction, &QuadOrientation, motorPercentages);
	}

	fclose(fPointer);
	return 1;
}


static void Init(void)
{
	Pid_Init();

	for (int i = 0; i < 4; i++)
	{
		motorPercentages[i] = 0;
	}

	QuadOrientation.xAccAngle = 0;
	QuadOrientation.yAccAngle = 0; 
	QuadOrientation.xGyroRate = 0;
	QuadOrientation.yGyroRate = 0;
	QuadOrientation.zGyroRate = 0;
	QuadOrientation.nSamples = 1;

	fPointer = fopen("SimulationResults/" OUTPUT_FILE_NAME, "w");
	fclose(fPointer);
	fPointer = fopen("SimulationResults/" OUTPUT_FILE_NAME, "a");
	fprintf(fPointer, "Time (s), Requested Angle (degrees), actualAngle (degrees), motor 0 power (percentage, taken from a bench mark of 50%%) ");
}

static void GetNextPilotInstruction(PilotResult_t *PilotInstruction)
{
	static int i;

	PilotInstruction->throttlePercentage = THROTTLE_POWER;
	PilotInstruction->xPercentage = CURRENT_TEST[i];
	PilotInstruction->yPercentage = 0;
	PilotInstruction->zPercentage = 0;

	i++;

}

static void OutputToFile(PilotResult_t *PilotInstruction, SensorResults_t *QuadOrientation, float *motorPercentages)
{
	static float time;

	fprintf(fPointer, "%s %f %s %f %s %f %s %f", "\n", time, ",", (PilotInstruction->xPercentage * 0.9), ",", QuadOrientation->xAccAngle, ",", motorPercentages[0] - 50.0f);

	time += CTRL_LOOP_PERIOD;
}

static void SimulateQuadPosition(float *motorPercentages, SensorResults_t *QuadOrientation)
{
	static float initialAngle;
	static float initialAngularVelocity;

	float angularAcceleration = GetAngularAcceleration(motorPercentages);

	QuadOrientation->xAccAngle = initialAngle + (initialAngularVelocity * CTRL_LOOP_PERIOD) + ( (1/2) * angularAcceleration * pow(CTRL_LOOP_PERIOD, 2) );
	QuadOrientation->xGyroRate = (QuadOrientation->xAccAngle - initialAngle) / CTRL_LOOP_PERIOD;


	initialAngle = QuadOrientation->xAccAngle;
	initialAngularVelocity += angularAcceleration * CTRL_LOOP_PERIOD;
}


static float GetAngularAcceleration(float *motorPercentages)
{
	return RADIANS_TO_DEGREES * 2.81 * (motorPercentages[0] - motorPercentages[2]);
}
