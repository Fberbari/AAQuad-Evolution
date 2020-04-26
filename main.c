/*
 * AAQuad-Evolution
 *
 * Firmware Developer : Anthony Berbari
 */

#include "Controller.h"

#include "Common.h"

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
	Controller_Init();

    while (1)
    {
    	Controller_Do();
    }

}
