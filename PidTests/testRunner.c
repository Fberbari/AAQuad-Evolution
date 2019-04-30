
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

#define SIZE_OF_DELAY		3

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static FILE *fPointer;

static float motorPercentages[4];
static SensorResults_t QuadOrientation;

static PilotResult_t PilotInstruction;

static float gyroNoisePropsOff[150] = {-0.1376667 ,0.014 ,0.11025 ,0.1044167 ,0.35525 ,0.14525 ,0.05191668 ,0.23275 ,3.434784E+19 ,2.289856E+19 ,0.23275 ,0.32025 ,0.3348334 ,0.06650001 ,0.6184792 ,0.02056248 ,0.02056248 ,0.005979143 ,0.02931248 ,0.04243748 ,0.02056248 ,-0.08443752 ,-0.1019375 ,-0.1077709 ,0.08181248 ,0.003062475 ,-0.7698541 ,0.01181247 ,0.01181247 ,0.07014581 ,0.02931248 ,-0.03193752 ,-0.04652086 ,0.02056248 ,-0.03193752 ,-0.09902086 ,0.09056249 ,0.05118748 ,-0.01735419 ,-0.1719375 ,0.289625 ,0.294 ,0.1365 ,0.154 ,0.07233334 ,0.04025 ,0.07962501 ,0.1015 ,-0.441 ,-0.3885 ,-0.1106875 ,-0.09318752 ,0.1693125 ,0.03368748 ,0.01472914 ,-0.1369375 ,-0.1500625 ,-0.09902086 ,-0.1106875 ,-0.1281875 ,-0.09610418 ,-259.8545 ,-4.975693 ,-14.06111 ,2.225563 ,-18.11381 ,-9.721105 ,1.901812 ,6.513062 ,8.344729 ,3.424313 ,-1.138813 ,-1.189854 ,-1.388188 ,2.807437 ,4.270146 ,-6.498188 ,-5.820063 ,-3.949021 ,-1.908813 ,0.8518125 ,-0.8763124 ,-1.624437 ,1.350562 ,1.827437 ,1.604312 ,-1.615688 ,-1.283188 ,-0.7786043 ,0.8343125 ,0.3968125 ,0.008895834 ,-0.2331875 ,0.04681247 ,0.1255625 ,-0.1806875 ,-0.2638125 ,-0.2390209 ,0.3268125 ,0.2524375 ,0.1051458 ,-0.08443752 ,-0.01881252 ,0.02639581 ,0.02931248 ,0.06431248 ,0.03806248 ,-0.07568753 ,-0.04506252 ,-0.02027085 ,-0.01225002 ,-0.01443752 ,-0.08881252 ,-0.08735418 ,-0.07568753 ,-0.03193753 ,-0.03485419 ,0.01181247 ,-0.001312524 ,0.0001458104 ,-0.05818752 ,-0.1194375 ,-0.1136042 ,-0.1019375 ,-0.1063125 ,-0.1311042 ,100.0156 ,-8.725063 ,-15.53402 ,2.960562 ,-12.29506 ,29.3039 ,-1.974438 ,-1.860688 ,-1.604021 ,0.8868125 ,0.4886875 ,0.04389582 ,-0.2944375 ,-0.02318752 ,0.01764581 ,-0.4081875 ,-0.4081875 ,-0.3294375 ,-0.2331875 ,-0.05818752 ,-0.1194375 ,-0.1369375 ,-0.01443752 ,0.007437482};
static float gyroNoisePropsOn[250] = {11.03153 ,8.858617 ,6.9132 ,4.929867 ,1.246117 ,-2.382217 ,-2.45805 ,-5.698467 ,-7.821796 ,-35.04889 ,2.016117 ,-1.848467 ,-0.9268 ,0.2632 ,2.325283 ,3.267367 ,2.8532 ,2.156117 ,1.7157 ,0.276325 ,-0.5738834 ,-1.028883 ,-1.237425 ,-1.953467 ,-1.67055 ,-1.79305 ,-0.756175 ,0.1202833 ,1.02445 ,2.89695 ,-45.2018 ,4.301325 ,-0.8101333 ,-3.268883 ,-3.248466 ,-4.859925 ,-5.841383 ,-4.420967 ,-3.424925 ,-2.390967 ,-0.6934667 ,0.556325 ,1.8557 ,1.65445 ,1.16445 ,0.5782 ,0.39445 ,0.8086167 ,1.046325 ,0.6598667 ,-6.907425 ,-18.09138 ,3.069033 ,-4.202217 ,-3.241175 ,-1.40805 ,-1.32055 ,0.3390333 ,0.6715333 ,-0.2618 ,-0.6993 ,-0.9618 ,-0.7022167 ,-0.721175 ,-1.00555 ,-0.34055 ,-0.25305 ,-0.41055 ,-0.6643 ,-0.09846666 ,1.011325 ,184.1095 ,27.6857 ,0.6132002 ,1.39195 ,-1.641383 ,1.02445 ,-3.4293 ,-2.522217 ,-1.307425 ,-1.559717 ,-0.9122167 ,-0.69055 ,-1.25055 ,-1.291383 ,-2.113883 ,-2.549925 ,-2.6768 ,-2.1518 ,-2.453675 ,-1.3468 ,-1.095967 ,-3.131802 ,6.184032 ,10.03695 ,-2.201383 ,0.62195 ,10.33445 ,6.814034 ,4.364033 ,5.211325 ,2.007367 ,0.9457 ,0.4557 ,-0.852425 ,-1.125133 ,-1.253467 ,-1.970967 ,-1.6093 ,-1.53055 ,-1.1893 ,1.00695 ,2.6957 ,-3.503677 ,4.066534 ,6.429033 ,5.90695 ,0.8786166 ,-1.5043 ,-2.0643 ,-3.283467 ,-4.657217 ,-3.96305 ,-3.914925 ,-3.846383 ,-3.747216 ,-2.567425 ,-0.8480501 ,0.4527833 ,1.7857 ,3.322783 ,3.818617 ,3.312575 ,2.173617 ,-9.189724 ,-100.4537 ,6.4232 ,-0.4601331 ,2.7307 ,-2.80805 ,0.86695 ,1.2782 ,0.2515334 ,1.765283 ,1.89945 ,2.80945 ,3.856533 ,2.89695 ,0.1173666 ,-1.96805 ,-3.33305 ,-3.974716 ,-4.805967 ,-3.152217 ,-2.716175 ,-13.8243 ,7.691949 ,-14.58263 ,-5.17055 ,4.66445 ,-0.6467999 ,1.476533 ,5.8457 ,6.006117 ,8.6282 ,10.2032 ,9.0132 ,6.977367 ,4.966325 ,2.234867 ,-0.8072166 ,-5.1443 ,-7.668675 ,-9.813884 ,-13.1068 ,-14.6468 ,-20.83888 ,-5.004301 ,-9.37055 ,-5.654716 ,-2.005967 ,-0.27055 ,1.54945 ,3.7457 ,3.976117 ,4.364033 ,5.64445 ,6.542783 ,6.34445 ,6.102367 ,4.08695 ,1.106117 ,-1.16305 ,-1.3118 ,-1.676383 ,-1.53055 ,-0.34055 ,-0.5301316 ,-5.5818 ,2.44195 ,2.121117 ,2.917367 ,2.0307 ,0.9690334 ,0.6773667 ,0.250075 ,0.04153333 ,-0.3318 ,-2.164925 ,-3.4993 ,-4.966383 ,-7.41055 ,-6.970133 ,-4.48805 ,-1.281175 ,2.509033 ,7.479033 ,10.19445 ,10.34028 ,-4.326174 ,7.934032 ,0.7648667 ,-2.392425 ,0.6132 ,-2.498883 ,-4.660133 ,-4.982425 ,-6.135967 ,-5.220133 ,-1.68805 ,-0.2413833 ,2.637367 ,4.2532 ,3.847783 ,1.9607 ,2.253825 ,2.2582 ,2.981533 ,3.618825 ,0.8261169 ,16.16195 ,6.895699 ,7.829033 ,-3.748675 ,-3.3068 ,-0.5359667 ,-1.725967 ,-3.066175 ,-1.05805 ,-1.113467};

static float motorDelay1[SIZE_OF_DELAY];
static float motorDelay3[SIZE_OF_DELAY];

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

	for (int i = 0; i < SIZE_OF_DELAY; i++)
	{
		motorDelay1[i] = 0;
	}
	for (int i = 0; i < SIZE_OF_DELAY; i++)
	{
		motorDelay3[i] = 0;
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
	PilotInstruction->xPercentage = 0; 
	PilotInstruction->yPercentage = CURRENT_TEST[i];
	PilotInstruction->zPercentage = 0;

	i++;

}

static void OutputToFile(PilotResult_t *PilotInstruction, SensorResults_t *QuadOrientation, float *motorPercentages)
{
	static float time;

	fprintf(fPointer, "%s %f %s %f %s %f %s %f %s %f", "\n", time, ",", (PilotInstruction->yPercentage) * MAX_Y_THROW / 100.0f, ",", QuadOrientation->yAccAngle, ",", motorPercentages[1] - 50.0f, ",", motorPercentages[3] - 50.0f);

	time += CTRL_LOOP_PERIOD;
}

static void SimulateQuadPosition(float *motorPercentages, SensorResults_t *QuadOrientation)
{
	static int i;
	static float initialAngle;
	static float initialAngularVelocity;

	float angularAcceleration = GetAngularAcceleration(motorPercentages);

	QuadOrientation->yAccAngle = initialAngle + (initialAngularVelocity * CTRL_LOOP_PERIOD) + ( 0.5f * angularAcceleration * CTRL_LOOP_PERIOD * CTRL_LOOP_PERIOD );
	QuadOrientation->yGyroRate = initialAngularVelocity + CTRL_LOOP_PERIOD * angularAcceleration;


	QuadOrientation->yGyroRate += gyroNoisePropsOn[i];

	i++;


	initialAngle = QuadOrientation->yAccAngle; 
	initialAngularVelocity += angularAcceleration * CTRL_LOOP_PERIOD;
}


static float GetAngularAcceleration(float *motorPercentages)
{

	for (int i = SIZE_OF_DELAY - 1; i > 0; i-- )
		{
			motorDelay1[i] = motorDelay1[i-1];
			motorDelay3[i] = motorDelay3[i-1];
		}
		motorDelay1[0] = motorPercentages[1];
		motorDelay3[0] = motorPercentages[3];



	return RADIANS_TO_DEGREES * 2.81 * (motorDelay3[SIZE_OF_DELAY - 1] - motorDelay1[SIZE_OF_DELAY - 1]);
}