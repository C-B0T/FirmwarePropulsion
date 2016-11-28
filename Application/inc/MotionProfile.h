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

    void setPoint(float point, float time); // target and time in seconds
    float update(float time);               // time in seconds

protected:
    void init();

    void calculateCustomTrapezoidalProfile(float t);
    void calculateTrapezoidalProfile(float t);
    void calculatePolynomial5Profile(float t);

    float setPoint;
    float factor;

    float startTime;    // time in seconds
};


#endif /* MOTIONPROFILE_H_ */


