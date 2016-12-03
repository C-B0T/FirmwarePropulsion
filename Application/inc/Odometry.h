**
 * @file    Odometry.h
 * @author  Jeremy ROULLAND
 * @date    3 dec. 2016
 * @brief   Odometry is the use to estimate position
 */


#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "common.h"

#include "Encoders.h"

#include <math.h>


#define WD          41.1                // Wheel diameter      
#define WC          1.00000000000       // Wheel correction (Difference between the (referent) right and left wheel)       
#define ER          2048                // Encoder resolution
#define ADW         230.35              // Axial distance between wheels

#define _PI_        3.14159265359       // PI
#define _2_PI_      6.28318530718       // 2*PI

#define TICK_BY_MM  15.8612809466       // (ER/(_PI_*WD))
#define AWD_TICK    3653.64606604       // (ADW * TICK_BY_MM)


typedef struct robot
{
    float32_t X;
    float32_t Y;
    float32_t O;
    float32_t L;

    int16_t AngularVelocity;
    int16_t LinearVelocity;
    
    int16_t LeftVelocity;
    int16_t RigthVelocity;

    float16_t Xmm;
    float16_t Ymm;
    float16_t Odeg;
    float16_t Lmm;
} robot_t;


/**
 * Provides a singletron Odometry class
 *
 */
class Odometry {
public:    
    Odometry();
    Odometry(float32_t X, float32_t Y, float32_t O, float32_t L);
    ~Odometry();

    getRobot(robot_t * r);

    setXY(float32_t X, float32_t Y);
    setXO(float32_t X, float32_t O);
    setYO(float32_t Y, float32_t O);

    compute();

protected:
    robot_t robot;

};


#endif /* ODOMETRY_H_ */


