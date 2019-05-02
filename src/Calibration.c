#include "Calibration.h"
#include "SensorData.h"
#include "PilotInstructions.h"
#include "Pid.h"
#include "I2CDriver.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define ACC_SENSITIVITY 0.000123f		// ( 8md/digit is what is is supposed to be. Tests revealed 1g corresponds to 1130 or so which is consistant with the +- 4g sensitivity commanded)
#define GYRO_SENSITIVITY 0.00875f		// dps/digit

#define ACC_SLAVE_ADDRESS 0x19
#define DEFAULT_REGISTER_VALUE 0x0

#define GYRO_SLAVE_ADDRESS 0x6B

#define RADIANS_TO_DEGREES 57.296f	// ratio of 180/pi

#define DO_PILOT_CALIBRATION
#define DO_GYRO_CALIBRATION

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static CalibratedZeros_t CalibratedZeros;
static InitialAngles_t  InitialAngles;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

#ifdef DO_ACC_CALIBRATION

static int GetRawAccData(int16_t *rawAccXData, int16_t *rawAccYData, int16_t *rawAccZData);

#endif
#ifdef DO_GYRO_CALIBRATION

static int GetRawGyroData(int16_t *rawGyroXData, int16_t *rawGyroYData, int16_t *rawGyroZData);

#endif

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/


void Calibration_Init(void)
{
	CalibratedZeros.xGyroRate = 0;
	CalibratedZeros.yGyroRate = 0;
	CalibratedZeros.zGyroRate = 0;

	InitialAngles.x = 0;
	InitialAngles.y = 0;
}

void Calibration_Calibrate(void)
{

#ifdef	DO_GYRO_CALIBRATION	// TODO most of this should be a function in the sensorData module.

	int16_t rawGyroXData, rawGyroYData, rawGyroZData;

	for (int i = 0; i < 100; i++)
	{
		GetRawGyroData(&rawGyroXData, &rawGyroYData, &rawGyroZData);
		CalibratedZeros.xGyroRate += GYRO_SENSITIVITY * (float) (rawGyroXData);
		CalibratedZeros.yGyroRate += GYRO_SENSITIVITY * (float) (rawGyroYData);
		CalibratedZeros.zGyroRate += GYRO_SENSITIVITY * (float) (rawGyroZData);
	}
	CalibratedZeros.xGyroRate /= 100.0f;
	CalibratedZeros.yGyroRate /= 100.0f;
	CalibratedZeros.zGyroRate /= 100.0f;

#else

	CalibratedZeros.xGyroRate = 0.0f;
	CalibratedZeros.yGyroRate = 0.0f;
	CalibratedZeros.zGyroRate = 0.0f;

#endif

	SensorData_CalibrateZeros(&CalibratedZeros);


#ifdef 	DO_ACC_CALIBRATION

	int16_t rawAccXData, rawAccYData, rawAccZData;
	
	for (int i = 0; i < 100; i++)
	{
		GetRawAccData(&rawAccXData, &rawAccYData, &rawAccZData);
		float RawAccMagnitude = sqrtf(Square(rawAccXData) + Square(rawAccYData) + Square(rawAccZData));
		InitialAngles.x += RADIANS_TO_DEGREES * (float)asinf((float) rawAccXData / (float)RawAccMagnitude);
		InitialAngles.y += RADIANS_TO_DEGREES * (float)asinf((float) rawAccYData / (float)RawAccMagnitude);
	}

	InitialAngles.x /= 100.0f;
	InitialAngles.y /= 100.0f;

#else

	InitialAngles.x = 0.0f;
	InitialAngles.y = 0.0f;

#endif

	Pid_CalibrateInitialAngles(&InitialAngles);

#ifdef DO_PILOT_CALIBRATION

PilotInstructions_Calibrate();

#endif


}


#ifdef DO_GYRO_CALIBRATION

static int GetRawGyroData(int16_t *rawGyroXData, int16_t *rawGyroYData, int16_t *rawGyroZData)
{
	I2CDriver_Start(); 
	I2CDriver_SendSlaveAddressWrite(GYRO_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0xA8);						// X_low +auto increment
	I2CDriver_RepeatStart();
	I2CDriver_SendSlaveAddressRead(GYRO_SLAVE_ADDRESS);
	I2CDriver_WaitForNextReadByte();
	*rawGyroYData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawGyroYData |= (TWDR0 << 8);
	I2CDriver_WaitForNextReadByte();
	*rawGyroXData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawGyroXData |= (TWDR0 << 8);
	I2CDriver_WaitForNextReadByte();
	*rawGyroZData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawGyroZData |= (TWDR0 << 8);

	I2CDriver_EndDataRead();
	I2CDriver_Stop();

	return AAQUAD_SUCCEEDED;
}

#endif

#ifdef DO_ACC_CALIBRATION

static int GetRawAccData(int16_t *rawAccXData, int16_t *rawAccYData, int16_t *rawAccZData)
{
	I2CDriver_Start(); 
	I2CDriver_SendSlaveAddressWrite(ACC_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0xA8);						// X_low +auto increment
	I2CDriver_RepeatStart();
	I2CDriver_SendSlaveAddressRead(ACC_SLAVE_ADDRESS);
	I2CDriver_WaitForNextReadByte();
	*rawAccYData = TWDR0;				// dataSheet says this reg is x but on the axis I've defined, it's -y
	I2CDriver_WaitForNextReadByte();
	*rawAccYData |= (TWDR0 << 8);	
	I2CDriver_WaitForNextReadByte();
	*rawAccXData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawAccXData |= (TWDR0 << 8);
	I2CDriver_WaitForNextReadByte();
	*rawAccZData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawAccZData |= (TWDR0 << 8);
	I2CDriver_EndDataRead();
	I2CDriver_Stop();

	*rawAccXData = -1 * (*rawAccXData);
	*rawAccYData = -1 * (*rawAccYData);

	return AAQUAD_SUCCEEDED;
}

#endif