#include "Common.h"


/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput)
{
	return minOutput + ((num - minInput) * ((maxOutput - minOutput) / (maxInput - minInput)));
}

float Square(int16_t num)
{
	return (float) ((float) num) * ((float) num);
}
