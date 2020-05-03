/*
 * AAQuad-Evolution
 *
 * Firmware Developer : Anthony Berbari
 */

#include "Controller.h"
#include "Calibration.h"

#include "Common.h"

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
	Controller_Init();
    Calibration_Calibrate();

    while (1)
    {
    	Controller_DoYourThingAndFlyTheQuadITrustYou();
    }

}
