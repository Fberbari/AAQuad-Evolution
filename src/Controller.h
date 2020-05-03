#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "Common.h"

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before anything is attempted to be done with the module
*/
void Controller_Init(void);

/**
* Executes a cycle of controller.
* Meant to be called in an infinite loop.
*/
void Controller_DoYourThingAndFlyTheQuadITrustYou(void);


#endif // _CONTROLLER_H
