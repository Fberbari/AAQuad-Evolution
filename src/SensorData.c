#include "SensorData.h"
#include "I2CDriver.h"

#include <avr/io.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
#define ACC_SENSITIVITY 0.000123f	// ( 8md/digit is what is is supposed to be. Tests revealed 1g corresponds to 1130 or so which is consistant with the +- 4g sensitivity commanded)
#define GYRO_SENSITIVITY 0.00875f		// dps/digit

#define ACC_SLAVE_ADDRESS 0x19
#define DEFAULT_REGISTER_VALUE 0x0

#define GYRO_SLAVE_ADDRESS 0x6B

#define RADIANS_TO_DEGREES 57.296f	// ratio of 180/pi

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

CalibratedZeros_t CalibratedZeros;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static int GetRawAccData(int16_t *rawAccXData, int16_t *rawAccYData, int16_t *rawAccZData);
static int GetRawGyroData(int16_t *rawGyroXData, int16_t *rawGyroYData, int16_t *rawGyroZData);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void SensorData_Init(void)
{
	I2CDriver_Start(); 
	I2CDriver_SendSlaveAddressWrite(ACC_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0xA0);						// Control register 1 + autoIncrement
	I2CDriver_SendData(0x67);								// all axis enable data refresh rate is 200Hz
	I2CDriver_SendData(DEFAULT_REGISTER_VALUE);
	I2CDriver_SendData(DEFAULT_REGISTER_VALUE);
	I2CDriver_SendData(0x10);								// CR4  +- 4g and 8mg/digit, The value 32767 corresponds to 4 g's
	I2CDriver_Stop();


	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(GYRO_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0x20);						// Control register 1 + autoincrement
	I2CDriver_SendData(0x4F);
	I2CDriver_Stop();	


}


int SensorData_GetResult(SensorResults_t *SensorResults)
{ 
	int16_t rawAccXData, rawAccYData, rawAccZData;
	int16_t rawGyroXData, rawGyroYData, rawGyroZData;

	GetRawAccData(&rawAccXData, &rawAccYData, &rawAccZData);
	float RawAccMagnitude = sqrtf(Square(rawAccXData) + Square(rawAccYData) + Square(rawAccZData));
	SensorResults->xAccAngle += RADIANS_TO_DEGREES * (float) asinf((float) rawAccXData / (float)RawAccMagnitude);
	SensorResults->yAccAngle += RADIANS_TO_DEGREES * (float) asinf((float) rawAccYData / (float)RawAccMagnitude);

	GetRawGyroData(&rawGyroXData, &rawGyroYData, &rawGyroZData);
	SensorResults->xGyroRate += (GYRO_SENSITIVITY * (float) (rawGyroXData)) - CalibratedZeros.xGyroRate;
	SensorResults->yGyroRate +=(GYRO_SENSITIVITY * (float) (rawGyroYData)) - CalibratedZeros.yGyroRate;
	SensorResults->zGyroRate += (GYRO_SENSITIVITY * (float) (rawGyroZData)) - CalibratedZeros.zGyroRate;

	SensorResults->nSamples ++;

	return AAQUAD_SUCCEEDED;
}

void SensorData_SensorReset(void)
{
	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(GYRO_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0x20);						// Control register 1 + autoincrement
	I2CDriver_SendData(0x0);								// turn off all axis and power down
	I2CDriver_Stop();	


	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(GYRO_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0x20);						// Control register 1 + autoincrement
	I2CDriver_SendData(0x7F);								// all axis enable data refresh rate is 800Hz,
	I2CDriver_Stop();	
}

void SensorData_CalibrateZeros(CalibratedZeros_t *Zeros)
{
	CalibratedZeros = *Zeros;
}



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