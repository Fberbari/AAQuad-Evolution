#include "Calibration.h"
#include "Imu.h"
#include "PilotInstructions.h"
#include "Altitude.h"
#include "MahonyAHRS.h"
#include "Pid.h"

#include "Common.h"

#include <avr/eeprom.h>

/***********************************************************************************************************************
 * Defines
 **********************************************************************************************************************/

#define EEPROM_START_ADDRESS	(const uint8_t *) 0x0

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float initialAzimuth;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static bool CalibrationRequested(void);
static void WaitUntilPilotReady(void);
static void GetCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration);
static void SaveCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration);
static void GetInitialAzimuth(void);
static void LoadCalibration(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Calibration_Calibrate(void)
{

	if ( CalibrationRequested() )
	{

		WaitUntilPilotReady();

		PilotResult_t PilotCalibration = {0};
		float altitudeCalibration = 0.0f;

		GetCalibration(&PilotCalibration, &altitudeCalibration);
		SaveCalibration(&PilotCalibration, &altitudeCalibration);

	}

	GetInitialAzimuth();

	LoadCalibration();
}

bool CalibrationRequested(void)
{
	PilotResult_t InitialPilotInput;

	while ( PilotInstructions_ComputePilotResult(&InitialPilotInput) == AAQUAD_BUSY)
	{
		asm("nop");
	}

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

void GetCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration)
{
	PilotCalibration->xPercentage = 0.0f;
	PilotCalibration->yPercentage = 0.0f;
	PilotCalibration->zPercentage = 0.0f;

	*altitudeCalibration = 0;

	PilotResult_t tempPilotResult = {0};
	float tempAltitude;

	const uint8_t nSamplesForReliableAverage = 100;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		Altitude_BeginMeasurement();

		while ( PilotInstructions_ComputePilotResult(&tempPilotResult) == AAQUAD_BUSY )
		{
			asm("nop");
		}

		while ( Altitude_Get(&tempAltitude) == AAQUAD_BUSY )
		{
			asm("nop");
		}

		PilotCalibration->xPercentage += tempPilotResult.xPercentage;
		PilotCalibration->yPercentage += tempPilotResult.yPercentage;
		PilotCalibration->zPercentage += tempPilotResult.zPercentage;
		PilotCalibration->throttlePercentage += tempPilotResult.throttlePercentage;

		*altitudeCalibration += tempAltitude;
	}

	PilotCalibration->xPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->yPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->zPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->throttlePercentage /= (float) nSamplesForReliableAverage;

	*altitudeCalibration /= (float) nSamplesForReliableAverage;
}

void SaveCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration)
{
	eeprom_write_block( (const void *) PilotCalibration, (void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t) );
	eeprom_write_block( (const void *) altitudeCalibration, (void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(float) );
}

static void GetInitialAzimuth(void)
{
	ImuData_t ImuData = {0};
	EulerXYZ_t EulerAngles = {0};

	const int nSamplesForReliableAverage = 1000;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		Imu_BeginRead();

		while ( Imu_GetResult(&ImuData) == AAQUAD_BUSY)
		{
			asm("nop");
		}

		MahonyAHRSupdate(ImuData.gyrX, ImuData.gyrY, ImuData.gyrZ, ImuData.accX, ImuData.accY, ImuData.accZ, ImuData.magX, ImuData.magY, ImuData.magZ);

		quat2Euler(q0, q1, q2, q3, &EulerAngles);

		initialAzimuth += EulerAngles.psi;
	}

	initialAzimuth /= (float) nSamplesForReliableAverage;
}

void LoadCalibration(void)
{
	PilotResult_t PilotCalibration = {0};
	float altitudeCalibration;

	eeprom_read_block( (void *) &PilotCalibration, (const void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t));
	eeprom_read_block( (void *) &altitudeCalibration, (const void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(float));

	PilotCalibration.zPercentage -= ((initialAzimuth * 100.0f) / MAX_Z_THROW);
	PilotInstructions_LoadCalibration(&PilotCalibration);

	Altitude_LoadCalibration(altitudeCalibration);
}
