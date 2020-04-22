/*
 * AAQuad-Evolution
 *
 * Firmware Developer : Anthony Berbari
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "Controller.h"
#include "Calibration.h"

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
	sei();
	Controller_Init();
	Calibration_Calibrate();

    while (1)
    {
    	Controller_Do();
    }

}
