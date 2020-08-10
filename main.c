/*
 * AAQuad-Evolution
 *
 * Firmware Developer : Anthony Berbari
 */

#include "Controller.h"
#include "Calibration.h"

#include "Common.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
#define TIME_TO_SETTLE_MS 500

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

int main(void)
{
    _delay_ms(TIME_TO_SETTLE_MS);

	Controller_Init();
    Calibration_Calibrate();

    while (1)
    {
    	Controller_DoYourThingAndFlyTheQuadITrustYou();
    }

}
