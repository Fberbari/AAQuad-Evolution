#include "Common.h"


/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput)
{
	return minOutput + ((num - minInput) * ((maxOutput - minOutput) / (maxInput - minInput)));
}