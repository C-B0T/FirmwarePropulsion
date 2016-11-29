/**
 * @file    TrajectoryPlanning.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "TrajectoryPlanning.h"

TrajectoryPlanning::TrajectoryPlanning()
{
    init();
    
    linearMotionProfile  = new MotionProfile();
    angularMotionProfile = new MotionProfile();
}

TrajectoryPlanning::~TrajectoryPlanning()
{
    delete linearMotionProfile;
    delete angularMotionProfile;
}

void TrajectoryPlanning::init()
{
}

void TrajectoryPlanning::goLinear(float linear) // linear in meters
{
    robot r;
    Odometry_GetRobot(&r);
    float time = getTime(); // time in seconds
    
    linearSign = (linear < 0.0) ? -1.0 : 1.0;
    
    linearPosition->setPoint(abs(linear));
    startLinearPosition = r.L;
    startAngularPosition = r.O;
    startTime = time;
    
    endLinearPosition  = startLinearPosition + linear; 
    endAngularPosition = startAngularPosition;
    
    state = LINEAR;
}

void TrajectoryPlanning::goAngular(float angular) // angular in radian
{
    robot r;
    Odometry_GetRobot(&r);
    float time = getTime(); // time in seconds
    
    angularSign = (angular < 0.0) ? -1.0 : 1.0;

    angularPosition->setPoint(abs(angular));
    startLinearPosition = r.L;
    startAngularPosition = r.O;
    startTime = time;
    
    endLinearPosition  = startLinearPosition; 
    endAngularPosition = startAngularPosition + angular;

    state = ANGULAR;
}

void TrajectoryPlanning::stop()
{
    robot r;
    Odometry_GetRobot(&r);
    float time = getTime(); // time in seconds
    
    startLinearPosition = r.L;
    startAngularPosition = r.O;
    startTime = time;
    
    //TODO: To prepare S-curve deceleration
    
    state = STOP;
}

void TrajectoryPlanning::gotoXY(float X, float Y)
{
    struct robot r;
    Odometry_GetRobot(&r)

    float XX = pow((X - r.X_m),2);
    float YY = pow((Y - r.Y_m),2);

    float dX = X - r.X_m;   // meters
    float dY = Y - r.Y_m;   // meters

    endLinearPosition = sqrtl(XX + YY);	// meters
    endAngularPosition = atan2f(dY,dX); // radians
    
    startLinearPosition = r.L;
    startAngularPosition = r.O;

    state = LINEARPLAN;
    lstate = 1;
}

float TrajectoryPlanning::udpate()
{
    switch(state)
    {
        // Simple mouvements
        case LINEAR:
            calculateGoLinear();
            break;
        
        case ANGULAR:
            calculateGoAngular();
            break;
        
        case STOP:
            calculateStop();
            break;
        
        case KEEP:
            calculateKeepPosition();
            break;
        
        // semi-complex mouvements
        case LINEARPLAN:
            calculateLinearPlan();
            break;
        
        // complex mouvements
        case CURVEPLAN:
            calculateCurvePlan();
            break;

        default:
            break;
    }
}

void TrajectoryPlanning::calculateGoLinear()
{
    float time = getTime(); // time in seconds
    time -= startTime;

    float profile = linearPosition->update(time);
    
    suggestedLinearPosition = linearSign * profile;
    suggestedAngularPosition = endAngularPosition;
}

void TrajectoryPlanning::calculateGoAngular()
{
    float time = getTime(); // time in seconds
    time -= startTime;

    float profile = angularPosition->update(time);
    
    suggestedLinearPosition = endLinearPosition;
    suggestedAngularPosition = angularSign * profile;
}


void TrajectoryPlanning::calculateStop()
{
    //TODO: S-Curve decceleration on linear and angular
}

void TrajectoryPlanning::calculateLinearPlan()
{
    float ErrL = endLinearPosition - startLinearPosition;
    float ErrO = endAngularPosition - startAngularPosition;

    float time = getTime(); // time in seconds
    time -= startTime;

    switch (lstate)
    {
        case 1:
            if(ErrO > -_PI_/36.0 && Err0 < _PI_/36.0)   // +/-5°
            {
                lstate = 2;
                startTime = getTime();
            }
            break;

        case 2:
            if(abs(ErrL) > 0.01) // +/-10mm
            {
                lstate = 3;
            }
            break;

        case 3:
            // KeepPosition robot
            break;

        default:
            break;
    }
    
    switch (lstate)
    {
        case 1:
            float profile = angularPosition->update(time);
            suggestedLinearPosition = startLinearPosition;
            suggestedAngularPosition = profile;
            break;

        case 2:
            float profile = linearPosition->update(time);
            suggestedLinearPosition = profile; 
            suggestedAngularPosition = AngularPosition;
            break;

        case 3:
            suggestedLinearPosition = LinearPosition; 
            suggestedAngularPosition = AngularPosition;
            break;

        default:
            break;
    }
    
}


void TrajectoryPlanning::calculateCurvePlan()
{
}

