#include "Common.h"


/***********************************************************************************************************************
 * Code
 **********************************************************************************************************************/

float map(float num, float minInput, float maxInput, float minOutput, float maxOutput)
{
    return minOutput + ((num - minInput) * ((maxOutput - minOutput) / (maxInput - minInput)));
}

float Square(int16_t num)
{
    return (float) ((float) num) * ((float) num);
}

float Squaref(float num)
{
    return num * num;
}

float SignedSquaref(float num)
{
    if (num < 0.0f)
    {
        return (-1.0f) * Squaref(num);
    }
    else
    {
        return Squaref(num);
    }
}

void quat2Euler(float q0, float q1, float q2, float q3, EulerZYX_t *EulerAngles)
{
    EulerAngles->phi = atan2f(2*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
    EulerAngles->theta = asinf((-2.0f) * ((q1*q3) - (q0*q2)));
    EulerAngles->psi = atan2f(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
}

void gyro2EulerRates(EulerZYX_t *EulerAngles, float angVelX, float angVelY, float angVelZ, EulerRates_t *EulerRates)
{
    EulerRates->phiDot = (angVelX * (float) cosf(EulerAngles->psi) / (float) cosf(EulerAngles->theta)) - (angVelY * (float) sinf(EulerAngles->psi) / (float) cosf(EulerAngles->theta));
    EulerRates->thetaDot = (angVelX * (float) sinf(EulerAngles->psi)) + (angVelY * (float) cosf(EulerAngles->psi));
    EulerRates->psiDot = (angVelX * (-1.0f) * (float) tanf(EulerAngles->theta) * (float) cosf(EulerAngles->psi)) +(angVelY * (float) tanf(EulerAngles->theta) * (float) sinf(EulerAngles->psi)) + angVelZ;
}

