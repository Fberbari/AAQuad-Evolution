#include "Altitude.h"
#include "I2C.h"
#include "LowPassFilter.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define SPEED_OF_SOUND 340.0f  // m/s
#define MAX_16_BIT_VALUE 65535U
#define SECONDS_PER_TICK ((float)(64.0f / F_CPU))    // assuming timer has prescaler of 64
#define TRIG_PIN 1

#define I2C_BUS 1
#define ALTIMETER_SLAVE_ADDRESS 0x60
#define ALT_CTRL_REG1 0x26
#define ALT_OUT_P_MSB_REG 0x01
#define STANDBY_MODE_8_OS 0x98
#define ACTIVE_MODE_8_OS 0x99


#define TIMER_VAL_11_US (F_CPU  / 90000)
#if (TIMER_VAL_11_US > 255)
	#error "8 bit timer cannot count to 11uS before overflowing, CPU is too fast"
#endif

#define ULTRASONIC_FILTER_WINDOW_SIZE 5

#define SENSOR_HANDOFF_ALTITUDE 3.0f   // altitude at which we stop using the ultrasonic and start using the altimeter

union AltimeterAltitudeRepresentation
{
    uint8_t byte[3];
    int32_t Q16point4;
};


/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float ultrasonicMeasuredAltitude;
static bool ultrasonicDataReady;
static bool ultrasonicMeasInProgress;
static float ultrasonicSystematicError;
static LowPassFilter_t UltrasonicFilter;
static float mostRecentUltrasonicMeas;

static union AltimeterAltitudeRepresentation rawAltimeter;
static bool altimeterDataReady;
static bool altimeterFailure;
static bool altimeterMeasInProgress;
static float altimeterSystematicError;

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static void InitAltimeter(void);
static void InitUltrasonic(void);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Altitude_Init(void)
{
    InitUltrasonic();
    InitAltimeter();

    UltrasonicFilter = LowPassFilter_CreateFilter(ULTRASONIC_FILTER_WINDOW_SIZE);
}

void Altitude_LoadCalibration(float ultrasonicCalibration, float altimeterCalibration)
{
	ultrasonicSystematicError = ultrasonicCalibration;
    altimeterSystematicError = altimeterCalibration;
}

void Altitude_BeginMeasurement(void)
{
    if (! ultrasonicMeasInProgress)
    {
        TCNT0 = 0;
        PORTB |= (1 << TRIG_PIN);
        TCCR0B = (1 << CS00);   // Start timer 0 with no prescaler

        ultrasonicMeasInProgress = true;
    }

    if (! altimeterMeasInProgress)
    {
        I2C_Start(I2C_BUS);

        altimeterMeasInProgress = true;
    }

}

int Altitude_Get(float *altitude)
{
    static bool previousMeasDoneWithUltrasonic = true;

    if (altimeterFailure)   // TODO something better can probably be done, since we might still have an operational ultrasonic.
    {
        return AAQUAD_FAILED;
    }

    if (ultrasonicDataReady)
    {
        mostRecentUltrasonicMeas = LowPassFilter_Execute(UltrasonicFilter, (ultrasonicMeasuredAltitude - ultrasonicSystematicError));
    }


    if (mostRecentUltrasonicMeas < SENSOR_HANDOFF_ALTITUDE)
    {
        if (ultrasonicDataReady)
        {
            *altitude = mostRecentUltrasonicMeas;
            ultrasonicDataReady = false;

            previousMeasDoneWithUltrasonic = true;

            return AAQUAD_SUCCEEDED;
        }

        else
        {
            return AAQUAD_BUSY;
        }
    }

    else
    {
        if (altimeterDataReady)
        {

            float tmp = (float) (rawAltimeter.Q16point4 >> 4);  // the 4 lowest bits of the data register are reserved
            tmp /= 16.0f;   // conversion from Q16.4 fixed point to float
            *altitude = tmp - altimeterSystematicError;
            altimeterDataReady = false;


            if (previousMeasDoneWithUltrasonic)
            {
                previousMeasDoneWithUltrasonic = false;

                altimeterSystematicError += (*altitude - SENSOR_HANDOFF_ALTITUDE);

                *altitude = SENSOR_HANDOFF_ALTITUDE;

            }

            return AAQUAD_SUCCEEDED;
        }

        else
        {
            return AAQUAD_BUSY;
        }
    }
}

int Altitude_AltimeterGet(float *altitude)
{
    if (altimeterFailure)
    {
        return AAQUAD_FAILED;
    }

    if (altimeterDataReady)
    {
        float tmp = (float) (rawAltimeter.Q16point4 >> 4);  // the 4 lowest bits of the data register are reserved
        tmp /= 16.0f;   // conversion from Q16.4 fixed point to float
        *altitude = tmp;
        altimeterDataReady = false;

        return AAQUAD_SUCCEEDED;
    }

    return AAQUAD_BUSY;
}

int Altitude_UltrasonicGet(float *altitude)
{
    if (ultrasonicDataReady)
    {
        *altitude = ultrasonicMeasuredAltitude;
        ultrasonicDataReady = false;

        return AAQUAD_SUCCEEDED;
    }

    return AAQUAD_BUSY;
}

static void InitUltrasonic(void)
{
    // timer 0 used to manage the trigger pulse
    // interrupts are initialised but the counter is not started yet
    TIMSK0 = (1 << OCIE0A);
    OCR0A = TIMER_VAL_11_US;

    // timer 3 used to measure the length of the echo pulse (prescaler of 64)
    TCCR3B = ((1 << CS31) | (1 << CS30));

    DDRB |= (1 << TRIG_PIN);    // TODO I burned the output driver of the trig pin so the temporary solution is to swap the trig and echo pin. This is currently done in this file. Just do it on the pcb. Permanent fix is just to replace the microcontroller.
    PORTB &= ~(1 << TRIG_PIN);

    // pcint for the echo pin
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
}

static void InitAltimeter(void)
{
    I2C_Init(I2C_BUS);
    I2C_DisableInterrupt(I2C_BUS);
    I2C_Start(I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendSlaveAddressWrite(ALTIMETER_SLAVE_ADDRESS, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendSlaveRegister(ALT_CTRL_REG1, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendData(STANDBY_MODE_8_OS, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_RepeatStart(I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendSlaveAddressWrite(ALTIMETER_SLAVE_ADDRESS, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendSlaveRegister(ALT_CTRL_REG1, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_SendData(ACTIVE_MODE_8_OS, I2C_BUS);
    I2C_BlockUntilReady(I2C_BUS);
    I2C_Stop(I2C_BUS);
    I2C_EnableInterrupt(I2C_BUS);
}

ISR(TIMER0_COMPA_vect)  // completes the emission of the trig pulse.
{
    PORTB &= ~(1 << TRIG_PIN);
    TCCR0B = 0; // stop timer 0
}

ISR(PCINT0_vect)    // measures the length of the echo pulse.
{
    static volatile uint16_t previousTimestamp;

    volatile uint16_t thisTimestamp = TCNT3;

    if ((PINB & (1 << PINB0)) == 0)	// falling edge
    {
		volatile int32_t counter;

		if (thisTimestamp < previousTimestamp)  // timer wrapped around
		{
			counter = thisTimestamp + (MAX_16_BIT_VALUE - previousTimestamp);
		}
		else
		{
			counter = thisTimestamp - previousTimestamp;
		}

        volatile float secondsOfTravel = (float) counter * SECONDS_PER_TICK;
        float distance = (secondsOfTravel * SPEED_OF_SOUND) / 2.0f;
        ultrasonicMeasuredAltitude = distance;
        ultrasonicDataReady = true;
        ultrasonicMeasInProgress = false;
    }

	previousTimestamp = thisTimestamp;
}

ISR(TWI1_vect)  // interrupt based altimeter reading
{
    static uint8_t i;

    switch (I2C_STATUS_BUS_1)
    {
        case START_SUCCEEDED :
            I2C_SendSlaveAddressWrite(ALTIMETER_SLAVE_ADDRESS, I2C_BUS);
            break;

        case SLAVE_WRITE_SUCCEEDED :
            I2C_SendSlaveRegister(ALT_OUT_P_MSB_REG, I2C_BUS);
            break;

        case DATA_TRANSMIT_SUCCEEDED :
            I2C_RepeatStart(I2C_BUS);
            break;

        case REPEAT_START_SUCCEEDED :
            I2C_SendSlaveAddressRead(ALTIMETER_SLAVE_ADDRESS, I2C_BUS);
            break;

        case SLAVE_READ_SUCCEEDED :
            I2C_AskForAnotherByte(I2C_BUS);
            break;

        case SLAVE_SENT_NEXT_BYTE :
            rawAltimeter.byte[2-i] = I2C_Read(I2C_BUS);

            if(i == 2)
            {
                I2C_EndDataRead(I2C_BUS);
                i = 0;
            }
            else
            {
                I2C_AskForAnotherByte(I2C_BUS);
                i++;
            }
            break;

        case END_READ_SUCCEEDED :
            I2C_Stop(I2C_BUS);
            altimeterDataReady = true;
            altimeterMeasInProgress = false;
            break;

        default:
            altimeterMeasInProgress = false;
            altimeterFailure = true;
    }
}
