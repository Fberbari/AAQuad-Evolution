#include "Calibration.h"
#include "SensorData.h"
#include "PilotInstructions.h"
#include "Pid.h"
#include "Leds.h"
#include "Common.h"

#include <avr/eeprom.h>

/***********************************************************************************************************************
 * Defines
 **********************************************************************************************************************/

#define EEPROM_START_ADDRESS	(const uint8_t *) 0x0

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

bool CalibrationRequested(void);
void WaitUntilPilotReady(void);
void GetCalibration(PilotResult_t *PilotCalibration, SensorResults_t *SensorCalibration);
void SaveCalibration(PilotResult_t *PilotCalibration, SensorResults_t *SensorCalibration);
void LoadCalibration(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Calibration_Calibrate(void)
{

	if ( CalibrationRequested() )
	{
		Leds_SetLed0();
		
		WaitUntilPilotReady();

		PilotResult_t PilotCalibration = {0};
		SensorResults_t SensorCalibration = {0};

		GetCalibration(&PilotCalibration, &SensorCalibration);
		SaveCalibration(&PilotCalibration, &SensorCalibration);
		
		Leds_ClearLed0();
	}

	LoadCalibration();

	float initialXAngle, initialYAngle;
	SensorData_GetInitialAngles(&initialXAngle, &initialYAngle);
	Pid_SetIntialAngles(initialXAngle, initialYAngle);
}

bool CalibrationRequested(void)
{
	PilotResult_t InitialPilotInput;

	while ( PilotInstructions_ComputePilotResult(&InitialPilotInput) == AAQUAD_BUSY)
	{
		asm("nop");
	}

	// TODO failure case ?

	if(InitialPilotInput.throttlePercentage > 50.0f)
	{
		return true;
	}

	return false;
}

void WaitUntilPilotReady(void)
{

	PilotResult_t PilotInput;

	do{
		
		while ( PilotInstructions_ComputePilotResult(&PilotInput) == AAQUAD_BUSY)
		{
			asm("nop");
		}
		
	} while ( PilotInput.throttlePercentage > MAX_VALUE_NO_PROP_SPIN);


}

void GetCalibration(PilotResult_t *PilotCalibration, SensorResults_t *SensorCalibration)
{

	SensorCalibration->xAccAngle = 0.0f;
	SensorCalibration->yAccAngle = 0.0f;
	SensorCalibration->xGyroRate = 0.0f;
	SensorCalibration->yGyroRate = 0.0f;
	SensorCalibration->zGyroRate = 0.0f;

	PilotCalibration->xPercentage = 0.0f;
	PilotCalibration->yPercentage = 0.0f;
	PilotCalibration->zPercentage = 0.0f;

	PilotResult_t tempPilotResult = {0};
	SensorResults_t tempSensorResult = {0};

	const uint8_t nSamplesForReliableAverage = 100;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		while ( PilotInstructions_ComputePilotResult(&tempPilotResult) == AAQUAD_BUSY)
		{
			asm("nop");
		}

		SensorData_GetResult(&tempSensorResult);

		SensorCalibration->xAccAngle += tempSensorResult.xAccAngle;
		SensorCalibration->yAccAngle += tempSensorResult.yAccAngle;
		SensorCalibration->xGyroRate += tempSensorResult.xGyroRate;
		SensorCalibration->yGyroRate += tempSensorResult.yGyroRate;
		SensorCalibration->zGyroRate += tempSensorResult.zGyroRate;

		PilotCalibration->xPercentage += tempPilotResult.xPercentage;
		PilotCalibration->yPercentage += tempPilotResult.yPercentage;
		PilotCalibration->zPercentage += tempPilotResult.zPercentage;

	}

	SensorCalibration->xAccAngle /= (float) nSamplesForReliableAverage;
	SensorCalibration->yAccAngle /= (float) nSamplesForReliableAverage;
	SensorCalibration->xGyroRate /= (float) nSamplesForReliableAverage;
	SensorCalibration->yGyroRate /= (float) nSamplesForReliableAverage;
	SensorCalibration->zGyroRate /= (float) nSamplesForReliableAverage;

	PilotCalibration->xPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->yPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->zPercentage /= (float) nSamplesForReliableAverage;

}

void SaveCalibration(PilotResult_t *PilotCalibration, SensorResults_t *SensorCalibration)
{
	eeprom_write_block( (const void *) PilotCalibration, (void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t) );
	eeprom_write_block( (const void *) SensorCalibration, (void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(SensorResults_t) );
}

void LoadCalibration(void)
{
	PilotResult_t PilotCalibration = {0};
	SensorResults_t SensorCalibration = {0};

	eeprom_read_block( (void *) &PilotCalibration, (const void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t));
	eeprom_read_block( (void *) &SensorCalibration, (const void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(SensorResults_t));

	PilotInstructions_LoadCalibration(&PilotCalibration);
	SensorData_LoadCalibration(&SensorCalibration);

}
