#include "Calibration.h"
#include "Imu.h"
#include "PilotInstructions.h"
#include "Altitude.h"
#include "Pid.h"
#include "MahonyAHRS.h"
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

static bool CalibrationRequested(void);
static void WaitUntilPilotReady(void);
static void GetCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration, ImuData_t* ImuCalibration);
static void GetPilotCalibration(PilotResult_t *PilotCalibration);
static void GetAltitudeCalibration(float *altitudeCalibration);
static void GetImuCalibration(ImuData_t* ImuCalibration);
static void SaveCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration, ImuData_t* ImuCalibration);
static void LoadCalibration(void);
static float GetInitialAzimuth(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Calibration_Calibrate(void)
{
	_delay_ms(5000);
	
	if ( CalibrationRequested() )
	{
		Leds_SetLed1();
		WaitUntilPilotReady();
		Leds_ClearLed1();

		PilotResult_t PilotCalibration = {0};
		float altitudeCalibration = 0.0f;
		ImuData_t ImuCalibration = {0};

		GetCalibration(&PilotCalibration, &altitudeCalibration, &ImuCalibration);
		SaveCalibration(&PilotCalibration, &altitudeCalibration, &ImuCalibration);

	}

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

static void GetCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration, ImuData_t* ImuCalibration)
{
	GetPilotCalibration(PilotCalibration);
	GetAltitudeCalibration(altitudeCalibration);
	GetImuCalibration(ImuCalibration);
}

static void GetPilotCalibration(PilotResult_t *PilotCalibration)
{
	PilotResult_t tempPilotResult;
	const uint8_t nSamplesForReliableAverage = 100;

	PilotCalibration->xPercentage = 0.0f;
	PilotCalibration->yPercentage = 0.0f;
	PilotCalibration->zPercentage = 0.0f;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{

		while ( PilotInstructions_ComputePilotResult(&tempPilotResult) == AAQUAD_BUSY )
		{
			asm("nop");
		}

		PilotCalibration->xPercentage += tempPilotResult.xPercentage;
		PilotCalibration->yPercentage += tempPilotResult.yPercentage;
		PilotCalibration->zPercentage += tempPilotResult.zPercentage;
		PilotCalibration->throttlePercentage += tempPilotResult.throttlePercentage;
	}

	PilotCalibration->xPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->yPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->zPercentage /= (float) nSamplesForReliableAverage;
	PilotCalibration->throttlePercentage /= (float) nSamplesForReliableAverage;
}

static void GetAltitudeCalibration(float *altitudeCalibration)
{
	float tempAltitude;
	const uint8_t nSamplesForReliableAverage = 100;

	*altitudeCalibration = 0;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		Altitude_BeginMeasurement();

		while ( Altitude_Get(&tempAltitude) == AAQUAD_BUSY )
		{
			asm("nop");
		}

		*altitudeCalibration += tempAltitude;
	}

	*altitudeCalibration /= (float) nSamplesForReliableAverage;
}

static void GetImuCalibration(ImuData_t* ImuCalibration)
{

	const int nSamplesForReliableAverage = 100;
	const int timeBetweenMeasurements = 5; // ms
	float degreeOfRotation = 0.0f;
	ImuData_t TempImuData;
	int j = 0;
	int ledToggleCnt = 0;
	const float ledTimeToToggle = 0.25f;

	ImuCalibration->accX = 0.0f;
	ImuCalibration->accY = 0.0f;
	ImuCalibration->accZ = 0.0f;
	ImuCalibration->gyrX = 0.0f;
	ImuCalibration->gyrY = 0.0f;
	ImuCalibration->gyrZ = 0.0f;
	ImuCalibration->magX = 0.0f;
	ImuCalibration->magY = 0.0f;
	ImuCalibration->magZ = 0.0f;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		Imu_BeginRead();

		_delay_ms(5);

		Imu_GetResult(&TempImuData);

		ImuCalibration->gyrX += TempImuData.gyrX;
		ImuCalibration->gyrY += TempImuData.gyrY;
		ImuCalibration->gyrZ += TempImuData.gyrZ;
		ImuCalibration->accX += TempImuData.accX;
		ImuCalibration->accY += TempImuData.accY;
		ImuCalibration->accZ += TempImuData.accZ;

	}

	ImuCalibration->gyrX /= (float) nSamplesForReliableAverage;
	ImuCalibration->gyrY /= (float) nSamplesForReliableAverage;
	ImuCalibration->gyrZ /= (float) nSamplesForReliableAverage;
	ImuCalibration->accX /= (float) nSamplesForReliableAverage;
	ImuCalibration->accY /= (float) nSamplesForReliableAverage;
	ImuCalibration->accZ /= (float) nSamplesForReliableAverage;

	ImuCalibration->accZ -= 1000.0f;	// at calibration, Z needs to read 1g.

	Imu_LoadCalibration(ImuCalibration);	// We need a calibrated gyro to be able to calibrate the magnetometer, might aswell load in the acc calibration aswell.

	while (j < 360)
	{
		Imu_BeginRead();

		_delay_ms(timeBetweenMeasurements);

		Imu_GetResult(&TempImuData);

		degreeOfRotation += (timeBetweenMeasurements * 0.001f) * TempImuData.gyrZ * RAD_TO_DEGREE;

		if (degreeOfRotation > j)
		{
			ImuCalibration->magX += TempImuData.magX;
			ImuCalibration->magY += TempImuData.magY;
			ImuCalibration->magZ += TempImuData.magZ;

			j++;
		}

		if (ledToggleCnt > (ledTimeToToggle / (timeBetweenMeasurements * 0.001f)))
		{
			Leds_ToggleLed1();
			ledToggleCnt = 0;
		}

		ledToggleCnt++;
	}

	ImuCalibration->magX /= 360.0f;
	ImuCalibration->magY /= 360.0f;
	ImuCalibration->magZ /= 360.0f;

	Leds_ClearLed1();
}

static void SaveCalibration(PilotResult_t *PilotCalibration, float *altitudeCalibration, ImuData_t* ImuCalibration)
{
	eeprom_write_block( (const void *) PilotCalibration, (void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t) );
	eeprom_write_block( (const void *) altitudeCalibration, (void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(float) );
	eeprom_write_block( (const void *) ImuCalibration, (void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t) + sizeof (float)), sizeof(ImuData_t) );
}

void LoadCalibration(void)
{
	PilotResult_t PilotCalibration;
	float altitudeCalibration;
	ImuData_t ImuCalibration;

	eeprom_read_block( (void *) &PilotCalibration, (const void *) EEPROM_START_ADDRESS, sizeof(PilotResult_t));
	eeprom_read_block( (void *) &altitudeCalibration, (const void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t)), sizeof(float));
	eeprom_read_block( (void *) &ImuCalibration, (const void *) (EEPROM_START_ADDRESS + sizeof(PilotResult_t) + sizeof (float)), sizeof(ImuData_t));

	Altitude_LoadCalibration(altitudeCalibration);
	Imu_LoadCalibration(&ImuCalibration);

	float initialAzimuth = GetInitialAzimuth();
	PilotCalibration.zPercentage -= ((initialAzimuth * 100.0f) / MAX_Z_THROW);
	PilotInstructions_LoadCalibration(&PilotCalibration);
}

static float GetInitialAzimuth(void)
{
	float initialAzimuth = 0.0f;
	ImuData_t ImuData = {0};
	EulerZYX_t EulerAngles = {0};

	const int nSamplesForReliableAverage = 10000;

	for (int i = 0; i < nSamplesForReliableAverage; i++)
	{
		Imu_BeginRead();

		_delay_ms(5);

		Imu_GetResult(&ImuData);

		MahonyAHRSupdate(ImuData.gyrX, ImuData.gyrY, ImuData.gyrZ, ImuData.accX, ImuData.accY, ImuData.accZ, ImuData.magX, ImuData.magY, ImuData.magZ);

		quat2Euler(q0, q1, q2, q3, &EulerAngles);

		if (i > (nSamplesForReliableAverage * (3.0f / 4.0f)))	// The Mahony algorithm needs to cycle a bit before it starts tracking position properly.
		{
			initialAzimuth += EulerAngles.psi;
		}

	}

	initialAzimuth /= (float) (nSamplesForReliableAverage * (1.0f / 4.0f));

	return initialAzimuth;
}
