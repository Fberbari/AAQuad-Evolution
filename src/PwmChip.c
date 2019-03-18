#include "PwmChip.h"

#include "I2CDriver.h"
#include <avr/io.h>

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void encode_motors(uint8_t motor, float* motors, uint8_t* instruction);

static void InitMotors(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void PwmChip_Init(void)
{
	I2CDriver_Init();

	DDRB |= (1 << 2);	// set OE to 0;

	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(0x4F);
	I2CDriver_SendSlaveRegister(0xFE);	//pre scale register
	I2CDriver_SendData(121);	//prescaler that corresponds to a 50 Hz frequency
	I2CDriver_RepeatStart();
	I2CDriver_SendSlaveAddressWrite(0x4F);
	I2CDriver_SendSlaveRegister(0x0); // mode register 1
	I2CDriver_SendData(0x21); //clock on, autoincrement enable
	I2CDriver_Stop();

	InitMotors();

}

int PwmChip_Send(float *motors)
{

// this function will communicate over I2C to the pwmchip for final controll of the motors

	uint8_t instruction[2];
	
	encode_motors(1, motors, instruction);

	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(0x4F);
  	I2CDriver_SendSlaveRegister(0x06);		//LED0_ON_L
	I2CDriver_SendData(0);
	I2CDriver_SendData(0);
	I2CDriver_SendData(instruction[0]); //ON_L, ON_H, OFF_L, OFF_H
	I2CDriver_SendData(instruction[1]);
	I2CDriver_Stop();


	encode_motors(0, motors, instruction);
	

	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(0x4F);
	I2CDriver_SendSlaveRegister(0x16);		//LED4_ON_L
	I2CDriver_SendData(0);
	I2CDriver_SendData(0);
	I2CDriver_SendData(instruction[0]); //ON_L, ON_H, OFF_L, OFF_H
	I2CDriver_SendData(instruction[1]);
	I2CDriver_Stop();

	
	encode_motors(2, motors, instruction);



	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(0x4F);
	I2CDriver_SendSlaveRegister(0x2E);		//LED8_ON_L
	I2CDriver_SendData(0);
	I2CDriver_SendData(0);
	I2CDriver_SendData(instruction[0]); //ON_L, ON_H, OFF_L, OFF_H
	I2CDriver_SendData(instruction[1]);
	I2CDriver_Stop();


	encode_motors(3, motors, instruction);


	I2CDriver_Start();
	I2CDriver_SendSlaveAddressWrite(0x4F);
	I2CDriver_SendSlaveRegister(0x42);		//LED8_ON_L
	I2CDriver_SendData(0);
	I2CDriver_SendData(0);
	I2CDriver_SendData(instruction[0]); //ON_L, ON_H, OFF_L, OFF_H
	I2CDriver_SendData(instruction[1]);
	I2CDriver_Stop();

	return AAQUAD_SUCCEEDED;
}

static void encode_motors(uint8_t motor, float* motors, uint8_t* instruction)
{
	
	uint16_t temp = (uint16_t) motors[motor]*2.05f;	// the actualslope of this curve is 2.05
	
	temp += 205;	// 205 is the value corresponding to 0 for the esc
	
	instruction[0] = ( temp & 0xff );	// conserves only the low byte
	
	instruction[1] = ( temp >> 8);	// conserves only the high half-byte
	
}



static void InitMotors(void)
{
}