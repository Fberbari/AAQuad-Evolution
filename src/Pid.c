#include "Pid.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define kpX 30.0f
#define kdX 4.0f
#define kiX 0.2f

#define kpY 30.0f
#define kdY 4.0f
#define kiY 0.2f

#define kpZ 20.0f
#define kdZ 5.0f
#define kiZ 0.2f

#define kpA 10.0f
#define kdA 2.0f
#define kiA 0.1f

#define TS CTRL_LOOP_PERIOD

#define MOTOR_PERCENT_LEVEL_ALTITUDE_HOLD 60.0f

/***********************************************************************************************************************
 * Variables
 **********************************************************************************************************************/

static float xAccumulatedIntegral;
static float yAccumulatedIntegral;
static float zAccumulatedIntegral;
static float altitudeAccumulatedIntegral;

static float historicalAltitudeValue[3];

/***********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************/

static float ComputeHeadingPid(float desired, float actual, float actualRate, float kp, float ki, float kd, float *accumulatedIntegral);
static float ComputeZPid(float desired, float actual, float actualRate);
static float ComputeAltitudePid(float desired, float actual, EulerXYZ_t *EulerAngles);
static void ConstrainMotorRanges(float *motors);

/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

void Pid_Init(void)
{

}

int Pid_Compute(PilotResult_t *PilotResult, EulerXYZ_t *EulerAngles, EulerRates_t *EulerRates, float altitude, float *motors)
{
	if (PilotResult->throttlePercentage < MAX_VALUE_NO_PROP_SPIN)
	{
		for (int i = 0; i < 4; i++)
		{
			motors[i] = MOTOR_VALUE_NO_SPIN;
		}

		altitudeAccumulatedIntegral = 0.0f;
		xAccumulatedIntegral = 0.0f;
		yAccumulatedIntegral = 0.0f;
		zAccumulatedIntegral = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			historicalAltitudeValue[i] = 0.0f;
		}

		return AAQUAD_SUCCEEDED;
	}

    float targetXAngle = MAX_X_THROW * (PilotResult->xPercentage / 100.0f);
    float targetYAngle = MAX_Y_THROW * (PilotResult->yPercentage / 100.0f);
    float targetZAngle = MAX_Z_THROW * (PilotResult->zPercentage / 100.0f);
    float targetAltitude = MAX_ALTITUDE * ((PilotResult->throttlePercentage - MAX_VALUE_NO_PROP_SPIN)/ 100.0f);	// TODO map the ranges instead

    float rollPercent = ComputeHeadingPid(targetXAngle, EulerAngles->phi, EulerRates->phiDot, kpX, kiX, kdX, &xAccumulatedIntegral);
    float pitchPercent = ComputeHeadingPid(targetYAngle, EulerAngles->theta, EulerRates->thetaDot, kpY, kiY, kdY, &yAccumulatedIntegral);

    float yawPercent = ComputeZPid(targetZAngle, EulerAngles->psi, EulerRates->psiDot);

    float altitudePercent = ComputeAltitudePid(targetAltitude, altitude, EulerAngles);

    motors[0] = altitudePercent;
    motors[1] = altitudePercent;
    motors[2] = altitudePercent;
    motors[3] = altitudePercent;

	motors[0] += rollPercent;
	motors[1] += rollPercent;
    motors[2] -= rollPercent;
    motors[3] -= rollPercent;

	motors[0] -= pitchPercent;
    motors[1] += pitchPercent;
	motors[2] -= pitchPercent;
	motors[3] += pitchPercent;

	motors[0] += yawPercent;
    motors[1] -= yawPercent;
    motors[2] -= yawPercent;
	motors[3] += yawPercent;

	ConstrainMotorRanges(motors);

	return AAQUAD_SUCCEEDED;
}

static float ComputeHeadingPid(float desired, float actual, float actualRate, float kp, float ki, float kd, float *accumulatedIntegral)
{
    float error = desired - actual;

    *accumulatedIntegral += (TS * error); // TODO anti windup ?

    return ((kp * error) + (ki * (*accumulatedIntegral)) - (kd * actualRate));
}

static float ComputeZPid(float desired, float actual, float actualRate)
{
    float error = desired - actual;

    if (error > (float) M_PI)
    {
        error = (desired - actual) - (2.0f * (float) M_PI);
    }
    else if (error < ((float) -M_PI))
    {
        error = (2.0f * (float) M_PI) + desired - actual;
    }

    zAccumulatedIntegral += (TS * error); // TODO anti windup ?

    return ((kpZ * error) + (kiZ * (zAccumulatedIntegral)) - (kdZ * actualRate));
}

static float ComputeAltitudePid(float desired, float actual, EulerXYZ_t *EulerAngles)
{

    float error = desired - actual;
    float altitudeHoldValue = MOTOR_PERCENT_LEVEL_ALTITUDE_HOLD / ((float) cosf(EulerAngles->phi) * (float) cosf(EulerAngles->theta));

    historicalAltitudeValue[2] = historicalAltitudeValue[1];
    historicalAltitudeValue[1] = historicalAltitudeValue[0];
    historicalAltitudeValue[0] = actual;

    float derivative = ((3 * historicalAltitudeValue[0]) - (4 * historicalAltitudeValue[1]) + (historicalAltitudeValue[2])) / TS;

    altitudeAccumulatedIntegral += (TS * error); // TODO anti windup ?

    return (altitudeHoldValue + (kpA * error) + (kiA * altitudeAccumulatedIntegral) - (kdA * derivative));
}

static void ConstrainMotorRanges(float *motors)
{
    for(int i = 0; i < 4; i++)
    {
        if (motors[i] > 99.0f)
        {
            motors[i] = 99.0f;
        }
        else if (motors[i] < MOTOR_VALUE_NO_SPIN)
        {
            motors[i] = MOTOR_VALUE_NO_SPIN;
        }
    }
}
