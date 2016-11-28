/**
 * @file    TrajectoryPlanning.h
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Trajectory planning (Linear, Curve, ...)
 */


#ifndef TRAJECTORYPLANNING_H_
#define TRAJECTORYPLANNING_H_

#include <maths.h>

#include "Odometry.h"
#include "MotionProfile.h"

/**
 * Provides a trajectory generator
 *
 */
class TrajectoryPlanning {
public:    
    TrajectoryPlanning();
    ~TrajectoryPlanning();

    //Orders:
    void goLinear(float linear) // linear in meters
    void goAngular(float angular) // angular in radian
    void stop();
    void gotoXY(float X, float Y);    // X,Y in meters
    // others orders...

    float update();
    float getSuggestedLinearPosition();
    float getSuggestedAngularPosition();

protected:
    enum state_t {LINEAR, ANGULAR, STOP, KEEP, LINEARPLAN, CURVEPLAN};

    void init();

    void calculateGoLinear();
    void calculateGoAngular();
    void calculateStop();
    void calculateLinearPlan();
    void calculateCurvePlan();

    int state;
    int lstate;
    
    float startTime;
    float startLinearPosition;  // 
    float startAngularPosition; // 

    float endLinearPosition;     // Linear Position Target
    float endAngularPosition;    // Linear Angular Target

    float linearSign;
    float angularSign;

    float suggestedLinearPosition;
    float suggestedAngularPosition;
    
    MotionProfile *angularMotionProfile;
    MotionProfile *libearMotionProfile;
};


#endif /* TRAJECTORYPLANNING */


