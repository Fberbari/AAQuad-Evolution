#include "Imu.h"
#include "SPI.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define TIME_BETWEEN_WRITES_MS 50

#define READ_BIT 0x80
#define WRITE_BIT 0x00
#define DUMMY_BYTE 0x00

#define DATA_REG 0x04
#define STATUS_REG 0x1B
#define CMD_REG 0x7E
#define PMU_STATUS_REG 0x03
#define ACC_CONF_REG 0x40
#define ACC_RANGE_REG 0x41
#define GYR_CONF_REG 0x42
#define GYR_RANGE_REG 0x43
#define MAG_CONF_REG 0x44
#define MAG_IF_0_REG 0x4C
#define MAG_IF_1_REG 0x4D
#define MAG_IF_2_REG 0x4E
#define MAG_IF_3_REG 0x4F
#define MAG_REPZ_REG 0x52
#define MAG_REPXY_REG 0x51
#define MAG_MODE_REG 0x4B

#define ACC_NORMAL_MODE_CMD 0x11
#define GYRO_NORMAL_MODE_CMD 0x15
#define MAG_NORMAL_MODE_CMD 0x19

#define ACC_RANGE_8G 0x08
#define ACC_ODR_200_OSR4 0x09	// samples are taken at 200 hz but every 4 are averaged.
#define GYRO_RANGE_1000	 0x01
#define GYRO_ODR_200_OSR4 0x09
#define MAG_SETUP_EN 0x80
#define MAG_SETUP_DIS 0x00
#define REP_XY_REGULAR_PRESET 0x04
#define REP_Z_REGULAR_PRESET 0x0E
#define MAG_IF_3_DATA_MODE 0x02
#define MAG_IF_2_DATA_MODE 0x4C
#define MAG_IF_1_DATA_MODE 0x42
#define MAG_REFRESH_50_HZ 0x07
#define MAG_SLEEP_MODE 0x01

#define MAG_REG_WRITTEN_BIT 2

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static uint8_t rawImuData[20];
static bool dataReady;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void SetAllPowerModesToNormal(void);
static void ConfigAcc(void);
static void ConfigGyro(void);
static void ConfigMag(void);
static void	SetMagConfig(uint8_t regAddr, uint8_t data);
static void PrepareMagForDataMode(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Imu_Init(void)
{
	SPI_Init();
	SetAllPowerModesToNormal();
	ConfigAcc();
	ConfigGyro();
	ConfigMag();
}

void Imu_BeginRead(void)
{
	SPI_BeginTransaction();
	SPI_Write(READ_BIT | DATA_REG);
}

int Imu_GetResult(ImuData_t *imuData)
{
	if (! dataReady)
	{
		return AAQUAD_BUSY;
	}

	imuData->magX = *((uint16_t *) ((void *) (&(rawImuData[0]))));
	imuData->magY = *((uint16_t *) ((void *) (&(rawImuData[2]))));
	imuData->magZ = *((uint16_t *) ((void *) (&(rawImuData[4]))));
	imuData->gyrX = *((uint16_t *) ((void *) (&(rawImuData[8]))));
	imuData->gyrY = *((uint16_t *) ((void *) (&(rawImuData[10]))));
	imuData->gyrZ = *((uint16_t *) ((void *) (&(rawImuData[12]))));
	imuData->accX = *((uint16_t *) ((void *) (&(rawImuData[14]))));
	imuData->accY = *((uint16_t *) ((void *) (&(rawImuData[16]))));
	imuData->accZ = *((uint16_t *) ((void *) (&(rawImuData[18]))));

	dataReady = false;

	return AAQUAD_SUCCEEDED;
}

static void SetAllPowerModesToNormal(void)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | CMD_REG);
	SPI_ReadWriteBlocking(ACC_NORMAL_MODE_CMD);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | CMD_REG);
	SPI_ReadWriteBlocking(GYRO_NORMAL_MODE_CMD);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | CMD_REG);
	SPI_ReadWriteBlocking(MAG_NORMAL_MODE_CMD);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);
}

static void ConfigAcc(void)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | ACC_RANGE_REG);
	SPI_ReadWriteBlocking(ACC_RANGE_8G);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | ACC_CONF_REG);
	SPI_ReadWriteBlocking(ACC_ODR_200_OSR4);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);
}

static void ConfigGyro(void)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | GYR_RANGE_REG);
	SPI_ReadWriteBlocking(GYRO_RANGE_1000);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | GYR_CONF_REG);
	SPI_ReadWriteBlocking(ACC_ODR_200_OSR4);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);
}

static void ConfigMag(void)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_0_REG);
	SPI_ReadWriteBlocking(MAG_SETUP_EN);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SetMagConfig(MAG_MODE_REG, MAG_SLEEP_MODE);
	SetMagConfig(MAG_REPXY_REG, REP_XY_REGULAR_PRESET);
	SetMagConfig(MAG_REPZ_REG, REP_Z_REGULAR_PRESET);

	PrepareMagForDataMode();

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_CONF_REG);
	SPI_ReadWriteBlocking(MAG_REFRESH_50_HZ);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_0_REG);
	SPI_ReadWriteBlocking(MAG_SETUP_DIS);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

}

static void	SetMagConfig(uint8_t regAddr, uint8_t data)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_3_REG);
	SPI_ReadWriteBlocking(data);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_2_REG);
	SPI_ReadWriteBlocking(regAddr);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

}

static void PrepareMagForDataMode(void)
{
	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_3_REG);
	SPI_ReadWriteBlocking(MAG_IF_3_DATA_MODE);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_2_REG);
	SPI_ReadWriteBlocking(MAG_IF_2_DATA_MODE);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);

	SPI_BeginTransaction();
	SPI_ReadWriteBlocking(WRITE_BIT | MAG_IF_1_REG);
	SPI_ReadWriteBlocking(MAG_IF_1_DATA_MODE);
	SPI_EndTransaction();

	_delay_ms(TIME_BETWEEN_WRITES_MS);
}

ISR(SPI1_STC_vect)	// spi transfer complete
{
	static int8_t numBytesCaptured = -1;

	if(numBytesCaptured != -1)
	{
		rawImuData[numBytesCaptured] = SPI_Read();
		numBytesCaptured ++;

		if(numBytesCaptured == 20)
		{
			dataReady = true;
			numBytesCaptured = -1;
			SPI_EndTransaction();
		}
		else
		{
			SPI_Write(DUMMY_BYTE);
		}
	}
	else
	{	// first call after we sent the reg address.
		// Data is not available yet.
		numBytesCaptured ++;
		SPI_Write(DUMMY_BYTE);
	}
}
