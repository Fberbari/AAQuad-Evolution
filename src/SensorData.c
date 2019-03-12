#include "SensorData.h"
#include "I2CDriver.h"

#include <avr/io.h>
#include <math.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define ACC_SENSITIVITY 0.000123f	// ( 8md/digit is what is is supposed to be. Tests revealed 1g corresponds to 1130 or so which is consistant with the +- 4g sensitivity commanded)

#define ACC_SLAVE_ADDRESS 0x19
#define DEFAULT_REGISTER_VALUE 0x0

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static int GetRawData(uint16_t *rawAccXData, uint16_t *rawAccYData, uint16_t *rawAccZData);
static float Square(uint16_t num);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void SensorData_Init(void)
{
	I2CDriver_Start(); 
	I2CDriver_SendSlaveAddressWrite(ACC_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0xA0);				// Control register 1 + autoincrement
	I2CDriver_SendData(0x67);						// all axis enable data refresh rate is 200Hz
	I2CDriver_SendData(DEFAULT_REGISTER_VALUE);		// CR2 default values
	I2CDriver_SendData(DEFAULT_REGISTER_VALUE);		// CR3 default values
	I2CDriver_SendData(0x10);						// CR4  +- 4g and 8mg/digit, The value 32767 corresponds to 4 g's
	I2CDriver_Stop();
}


int SensorData_GetResult(SensorResults_t *SensorResults)
{ 
	uint16_t rawAccXData, rawAccYData, rawAccZData;

	GetRawData(&rawAccXData, &rawAccYData, &rawAccZData);

	float RawAccMagnitude = sqrtf(Square(rawAccXData) + Square(rawAccYData) + Square(rawAccZData));		// in no particular unit
	float RealAccMagnitude = RawAccMagnitude * ACC_SENSITIVITY;															// in g's


	SensorResults->xAngle = asinf((float) rawAccXData / RawAccMagnitude);
	SensorResults->yAngle = asinf((float) rawAccYData / RawAccMagnitude);

	return AAQUAD_SUCCEEDED;
}

static int GetRawData(uint16_t *rawAccXData, uint16_t *rawAccYData, uint16_t *rawAccZData)
{
	I2CDriver_Start(); 
	I2CDriver_SendSlaveAddressWrite(ACC_SLAVE_ADDRESS);
	I2CDriver_SendSlaveRegister(0xA8);						// X_low +auto increment
	I2CDriver_RepeatStart();
	I2CDriver_SendSlaveAddressRead(ACC_SLAVE_ADDRESS);
	I2CDriver_WaitForNextReadByte();
	*rawAccXData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawAccXData |= (TWDR0 << 8);	
	I2CDriver_WaitForNextReadByte();
	*rawAccYData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawAccYData |= (TWDR0 << 8);
	I2CDriver_WaitForNextReadByte();
	*rawAccZData = TWDR0;
	I2CDriver_WaitForNextReadByte();
	*rawAccZData |= (TWDR0 << 8);
	I2CDriver_EndDataRead();
	I2CDriver_Stop();

	return AAQUAD_SUCCEEDED;
}

static float Square(uint16_t num)
{
	return (float) ((float) num) * ((float) num);
}
