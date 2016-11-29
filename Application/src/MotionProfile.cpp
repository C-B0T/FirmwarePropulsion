/**
 * @file    MotionProfile.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
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
}


void MotionProfile::setPoint(float point)
{
    setPoint = point;

    factor = setPoint / 1.0;
}

float MotionProfile::udpate(float time)
{
    float t;
    float r;

    t = time;
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
    return 10.0*pow(t,3)-15*pow(t,4)+6*pow(t,5);
}


