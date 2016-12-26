/**
 * @file    MotionProfile.h
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#ifndef MOTIONPROFILE_H_
#define MOTIONPROFILE_H_


/**
 * Provides a motion profile generator
 *
 */
class MotionProfile {
public:    
    MotionProfile();
    ~MotionProfile();

    void SetsetPoint(float point); // target and time in seconds
    float update(float time);   // time in seconds

    bool isFinished();

protected:
    void init();

    bool finished;

    float calculateCustomTrapezoidalProfile(float t);
    float calculateTrapezoidalProfile(float t);
    float calculatePolynomial5Profile(float t);

    float setPoint;
    float factor;
};


#endif /* MOTIONPROFILE_H_ */


