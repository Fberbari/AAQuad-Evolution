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
#define ACC_ODR_800_OSR4 0x0A	// samples are taken at 800 hz but every 4 are averaged. TODO the magnetometer is really difficult. And I suspect there is still a lot wrong with it.
#define GYRO_RANGE_1000	 0x01
#define GYRO_ODR_800_OSR4 0x0A
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

#define GYRO_RANGE_1000_FACTOR 1879.3f	// LSB/rad/s

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

int Imu_GetResult(ImuData_t *ImuData)
{
	if (! dataReady)
	{
		return AAQUAD_BUSY;
	}

	int16_t *intImuDataPtr = (int16_t *) rawImuData;

	int16_t magX = *(&(intImuDataPtr[0]));
	int16_t magY = *(&(intImuDataPtr[1]));
	int16_t magZ = *(&(intImuDataPtr[2]));
	int16_t gyrX = *(&(intImuDataPtr[4]));
	int16_t gyrY = *(&(intImuDataPtr[5]));
	int16_t gyrZ = *(&(intImuDataPtr[6]));
	int16_t accX = *(&(intImuDataPtr[7]));
	int16_t accY = *(&(intImuDataPtr[8]));
	int16_t accZ = *(&(intImuDataPtr[9]));

	ImuData->magX = (float) magX;
	ImuData->magY = (float) magY;
	ImuData->magZ = (float) magZ;
	ImuData->accX = (float) accX;
	ImuData->accY = (float) accY;
	ImuData->accZ = (float) accZ;
	ImuData->gyrX = (float) gyrX / GYRO_RANGE_1000_FACTOR;
	ImuData->gyrY = (float) gyrY / GYRO_RANGE_1000_FACTOR;
	ImuData->gyrZ = (float) gyrZ / GYRO_RANGE_1000_FACTOR;

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
	SPI_ReadWriteBlocking(ACC_ODR_800_OSR4);
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
	SPI_ReadWriteBlocking(ACC_ODR_800_OSR4);
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
