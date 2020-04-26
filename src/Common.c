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

void quat2Euler(float q0, float q1, float q2, float q3, EulerXYZ_t *EulerAngles)
{
    float sinPh_cosT = (-1) * 2 * ((q2 * q3) - (q0 * q1));
    float cosPh_cosT = (Squaref(q0) +Squaref(q3)) - (Squaref(q1) + Squaref(q2));
    float sinT = 2 * ((q0 * q2) + (q3 * q1));
    float sinPs_cosT = (-1) * 2 * ((q1 * q2) - (q0 * q3));
    float cosPs_cosT = (Squaref(q0) + Squaref(q1)) - (Squaref(q2) + Squaref(q3));

    EulerAngles->phi = atan2f(sinPh_cosT, cosPh_cosT);
    EulerAngles->theta = asinf(sinT);
    EulerAngles->psi = atan2f(sinPs_cosT, cosPs_cosT);
}

void gyro2EulerRates(EulerXYZ_t *EulerAngles, float angVelX, float angVelY, float angVelZ, EulerRates_t *EulerRates)
{
    EulerRates->phiDot = (angVelX * (float) cosf(EulerAngles->psi) / (float) cosf(EulerAngles->theta)) - (angVelY * (float) sinf(EulerAngles->psi) / (float) cosf(EulerAngles->theta));
    EulerRates->thetaDot = (angVelX * (float) sinf(EulerAngles->psi)) + (angVelY * (float) cosf(EulerAngles->psi));
    EulerRates->psiDot = (angVelX * (-1.0f) * (float) tanf(EulerAngles->theta) * (float) cosf(EulerAngles->psi)) +(angVelY * (float) tanf(EulerAngles->theta) * (float) sinf(EulerAngles->psi)) + angVelZ;
}

