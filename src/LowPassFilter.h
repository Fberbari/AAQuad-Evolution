#ifndef _LOWPASSFILTER_H
#define _LOWPASSFILTER_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

typedef struct LowPassFilterStruct *LowPassFilter_t;	// forward declaration of the filter

#define MOVING_MEDIAN
//#define MOVING_AVERAGE

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Returns an instance of a filter.
* returns Null if system is out of memory
* param[in]		windowSize	the window size of the moving average that will be used with this filter.
* @return					A pointer to the filter handle or Null
*/
LowPassFilter_t LowPassFilter_CreateFilter(unsigned int windowSize);

/**
* Destroys the Filter and frees any memory allocated in the process.
* Safe to pass in a NULL pointer.
* @param[in]		Filter 			The handle of the filter the user wants to destroy.
*/
void LowPassFilter_DestroyFilter(LowPassFilter_t Filter);

/**
* Gives the user the filtered output.
* @param[in]		Filter 			the handle to the filter the user is using.
* @param[in]		dataPoint 		the next data point the user has to filter.
* @return			the filtered result
*/
float LowPassFilter_Execute(LowPassFilter_t Filter, float dataPoint);

#endif // _LOWPASSFILTER_H
