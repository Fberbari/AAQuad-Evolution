#include "PwmChip.h"
#include "I2C.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define PWM_SLAVE_ADDRESS 0x4F

#define PRESCALE_REG 0xFE
#define MODE_1_REG 0x00
#define LED0_ON_L_REG 0x06

#define PRESCALER_50_HZ 0x79
#define CLK_ON_AUTO_INCREMENT_ENABLE 0x21

union MotorPercentBytesRepresentation
{
	uint16_t bytes;
	uint8_t byte[2];
};

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static volatile bool I2CIsBusy = false;
static volatile bool I2CError = false;
static uint8_t I2CSendBytes[16] = {0};

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void SetOEPinAsOutput(void);
static void EnableOutput(void);
static void DisableOutput(void);
static void encodeMotor(float motorPercent, uint8_t* instruction);
static void InitMotorsToZero(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void PwmChip_Init(void)
{
	SetOEPinAsOutput();
	DisableOutput();

	I2C_Init();
	I2C_DisableInterrupt();

	I2C_Start();
	I2C_BlockUntilReady();
	I2C_SendSlaveAddressWrite(PWM_SLAVE_ADDRESS);
	I2C_BlockUntilReady();
	I2C_SendSlaveRegister(PRESCALE_REG);
	I2C_BlockUntilReady();
	I2C_SendData(PRESCALER_50_HZ);
	I2C_BlockUntilReady();
	I2C_RepeatStart();
	I2C_BlockUntilReady();
	I2C_SendSlaveAddressWrite(PWM_SLAVE_ADDRESS);
	I2C_BlockUntilReady();
	I2C_SendSlaveRegister(MODE_1_REG);
	I2C_BlockUntilReady();
	I2C_SendData(CLK_ON_AUTO_INCREMENT_ENABLE);
	I2C_BlockUntilReady();
	I2C_Stop();

	I2C_EnableInterrupt();

	InitMotorsToZero();
	EnableOutput();

}

int PwmChip_Send(float *motors)
{
	if (I2CIsBusy)
	{
		return AAQUAD_BUSY;
	}

	if (I2CError)
	{
		return AAQUAD_FAILED;
	}

	I2CIsBusy = true;

	// all non-mentioned bytes are 0
	encodeMotor(motors[0], &(I2CSendBytes[2]));
	encodeMotor(motors[1], &(I2CSendBytes[6]));
	encodeMotor(motors[2], &(I2CSendBytes[10]));
	encodeMotor(motors[3], &(I2CSendBytes[14]));

	I2C_Start();

	return AAQUAD_SUCCEEDED;
}

static void encodeMotor(float motorPercent, uint8_t* instruction)
{

	union MotorPercentBytesRepresentation motorBytes;

	motorBytes.bytes = (uint16_t) (motorPercent * 2.05f);

	motorBytes.bytes += 205;	// 205 is the value corresponding to 0 for the esc

	instruction[0] = motorBytes.byte[0];
	instruction[1] = motorBytes.byte[1];
}

static void InitMotorsToZero(void)
{
	float motors[4];
	for (int i = 0; i < 4; i++)
	{
		motors[i] = MOTOR_VALUE_NO_SPIN;
	}

	PwmChip_Send(motors);

	// just poll until the exchange completes
	// best to not move forward with initialisation until this is done.
	while(I2CIsBusy)
	{
		asm("nop");
	}

}

static void SetOEPinAsOutput(void)
{
	DDRC |= (1 << 2);
}

static void DisableOutput(void)
{
	PORTC |= (1 << 2);
}

static void EnableOutput(void)
{
	PORTC &= ~(1 << 2);
}

ISR(TWI0_vect)
{
	static int8_t i;

	switch (STATUS)
	{
		case START_SUCCEEDED :
			I2C_SendSlaveAddressWrite(PWM_SLAVE_ADDRESS);
			break;

		case SLAVE_WRITE_SUCCEEDED :
			I2C_SendSlaveRegister(LED0_ON_L_REG);
			break;

		case DATA_TRANSMIT_SUCCEEDED :
			if(i == 16)
			{
				I2C_Stop();
				i = 0;
				I2CIsBusy = false;
			}
			else
			{
				I2C_SendData(I2CSendBytes[i]);
				i++;
			}
			break;

		default:
			I2CIsBusy = false;
			I2CError = true;
	}
}
