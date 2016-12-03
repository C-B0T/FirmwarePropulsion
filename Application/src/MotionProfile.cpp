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

    finished = false;
}

bool MotionProfile::isFinished()
{
    return finished;
}


void MotionProfile::setPoint(float point)
{
    setPoint = point;

    factor = setPoint / 1.0;

    finished = false;
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

    if(t >= 1.0)
        finished = true;
    else
        finished = false;


    return r;
}

float MotionProfile::calculateCustomTrapezoidalProfile(float t)
{
    
}

float MotionProfile::calculateTrapezoidalProfile(float t)
{
    float s = 0.0;

    if(t <= 0.25)    // [0 to 0.25]
    {
        //Acceleration
        s = 2*pow(t,2);
    }
    else if (t <= 0.75)    // ]0.25 to 0.75]
    {
        //Constant velocity
        s = t;
    }
    else if (t <= 1.0)  // ]0.75 to 1.0]
    {
        //Deceleration
        s = -1 + 4*t - 2*pow(t,2);
    }
    else
    {
        // !! Anormal condition
        t = 1.0;
        //Deceleration
        s = -1 + 4*t - 2*pow(t,2);
    }

    return s;
}

float MotionProfile::calculatePolynomial5Profile(float t)
{
    return 10.0*pow(t,3)-15*pow(t,4)+6*pow(t,5);
}


