/**
 * @file    MotionProfile.cpp
 * @author    Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief    Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "MotionProfile.h"

#include <maths.h>

MotionProfile::MotionProfile()
{
    init();
}

MotionProfile::~MotionProfile()
{
}

void MotionProfile::init()
{
    setPoint = 0.0;
    factor = 0.0;

    startTime = 0.0;
}


void MotionProfile::setPoint(float point, float time)
{
    setPoint = point;

    factor = setPoint / 1.0;

    startTime = time;
}

float MotionProfile::udpate(float time)
{
    float t;
    float r;

    t = time - startTime;
    t /= factor;

    if(t <= 1.0)
        r = calculatePolynomial5Profile(t);
    else
        r = 1.0;
    r *= factor;

    return r;
}

float MotionProfile::calculateCustomTrapezoidalProfile(float t)
{
    
}

float MotionProfile::calculateTrapezoidalProfile(float t)
{
    
}

float MotionProfile::calculatePolynomial5Profile(float t)
{
    return 10.0*pow(time,3)-15*pow(time,4)+6*pow(time,5);
}


