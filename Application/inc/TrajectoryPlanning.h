/**
 * @file    TrajectoryPlanning.h
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Trajectory planning (Linear, Curve, ...)
 */


#ifndef TRAJECTORYPLANNING_H_
#define TRAJECTORYPLANNING_H_

#include <math.h>

#include "Odometry.hpp"
#include "MotionProfile.h"

#define _PI_        3.14159265358979323846


/**
 * Provides a trajectory generator
 *
 */
class TrajectoryPlanning {
public:    
    TrajectoryPlanning();
    ~TrajectoryPlanning();

    //Orders:
    void goLinear(float linear)      // linear in meters
    void goAngular(float angular)    // angular in radian
    void stop();
    void gotoXY(float X, float Y);   // X,Y in meters
    int stallX(int stallMode);       // stallMode allow to choose side to side contact (upTable to backBot, upTable to frontBot, downTable to backBot, downTable to frontBot)
    int stallY(int stallMode);       // stallMode allow to choose side to side contact (leftTable to backBot, leftTable to frontBot, rightTable to backBot, rightTable to frontBot)
    // others orders...

    float update();
    float getSuggestedLinearPosition();
    float getSuggestedAngularPosition();

protected:
    enum state_t {FREE=0, LINEAR, ANGULAR, STOP, KEEP, LINEARPLAN, CURVEPLAN, STALLX, STALLY};

    void init();

    void calculateGoLinear();
    void calculateGoAngular();
    void calculateStop();
    void calculateLinearPlan();
    void calculateCurvePlan();
    void calculateStallX();
    void calculateStallY();

    float 

    int state;
    int lstate;

    int stallMode;
    
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


