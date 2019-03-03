#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

/**
* Initialises internal parameters
* Should be called once before annything is attempted to be done with the module
*/
void Controller_Init(void);

/**
* Initialises all modules that controller is in charge of handling.
* Should be called once, right after Controller_Init is called and before any other functions in this module or library are called.
*/
void Controller_InitAll(void);

/**
* Executes a cycle of controller.
* Meant to be calledin an infinite loop.
*/
void Controller_Do(void);


#endif // _CONTROLLER_H
