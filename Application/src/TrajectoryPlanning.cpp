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
	state = FREE;
	lstate = 0;

	stallMode = 0;
}

void TrajectoryPlanning::goLinear(float linear) // linear in meters
{
    robot r;
    Odometry_GetRobot(&r);
    float time = getTime(); // time in seconds
    
    linearSign = (linear < 0.0) ? -1.0 : 1.0;
    
    libearMotionProfile->setPoint(abs(linear));
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

    angularMotionProfile->setPoint(abs(angular));
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

int stallX(int stallMode)
{ 
	// TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
	//                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

    struct robot r;
    Odometry_GetRobot(&r)

    startLinearPosition = r.L;
    startAngularPosition = r.O;

	endLinearPosition = startLinearPosition - 0.0;
    endAngularPosition = 0.0;

    state = STALLX;
    lstate = 1;
}

int stallY(int stallMode)
{ 
	// TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
	//                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

    struct robot r;
    Odometry_GetRobot(&r)

    startLinearPosition = r.L;
    startAngularPosition = r.O;

	endLinearPosition = startLinearPosition - 0.0;
    endAngularPosition = _PI_/2.0;

    state = STALLY;
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

        // special mouvements
        case STALLX:
            calculateStallX(1);
            break;
        case STALLY:
            calculateStallY(1);
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
            float profile = angularMotionProfile->update(time);
            suggestedLinearPosition = startLinearPosition;
            suggestedAngularPosition = profile;
            break;

        case 2:
            float profile = linearMotionPosition->update(time);
            suggestedLinearPosition = profile; 
            suggestedAngularPosition = endAngularPosition;
            break;

        case 3:
            suggestedLinearPosition = endLinearPosition; 
            suggestedAngularPosition = endAngularPosition;
            break;

        default:
            break;
    }
    
}


void TrajectoryPlanning::calculateCurvePlan()
{
}


void TrajectoryPlanning::calculateStallX()
{
	float time = getTime();
	time -= startTime;

	//TODO:Add stallMode gestion

	switch (lstate)
	{
		case 1:
			if(angularMotionProfile->isFinished())
			{
				lstate = 2;
			}
			break;
		case 2:
			if(jackBackLeft && jackBackLeft)
			{
				lstate = 3;
			}
			break;
		case 3:
			// X axis and Angular are calibrated
			break;
		default:
			break;
	}

	switch (lstate)
	{
		case 1: // Rotate to 0 rad
			float profile = angularMotionProfile->update(time);
			suggestedLinearPosition = startLinearPosition;
			suggestedAngularPosition = profile;
			break;

		case 2:
            //TODO:Disable Angular asserv.
			//TODO:Back and wait both jacks
			break;

		case 3:
			//TODO:Modify X value in function of the mechanic 
			Odometry_SetX(0.0);
			Odometry_SetO(0.0);
			break;

		default:
			break;
	}

}

void TrajectoryPlanning::calculateStallY()
{
	float time = getTime();
	time -= startTime;

	//TODO:Add stallMode gestion

	switch (lstate)
	{
		case 1:
			if(angularMotionProfile->isFinished())
			{
				lstate = 2;
			}
			break;
		case 2:
			if(jackBackLeft && jackBackLeft)
			{
				lstate = 3;
			}
			break;
		case 3:
			// Y axis and Angular are calibrated
			break;
		default:
			break;
	}

	switch (lstate)
	{
		case 1: // Rotate to pi/2 rad
			float profile = angularMotionProfile->update(time);
			suggestedLinearPosition = startLinearPosition;
			suggestedAngularPosition = profile;
			break;

		case 2:
            //TODO:Disable Angular asserv.
			//TODO:Back and wait both jacks
			break;

		case 3:
			//TODO:Modify Y value in function of the mechanic 
			Odometry_SetY(0.0);
			Odometry_SetO(0.0);
			break;

		default:
			break;
	}

}

