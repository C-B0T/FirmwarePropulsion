/**
 * @file    Odometry.hpp
 * @author  Jeremy ROULLAND
 * @date    3 dec. 2016
 * @brief   Odometry is the use to estimate position
 */


#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "common.h"
#include "Encoder.hpp"

#include <math.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define WD          41.1                // Wheel diameter
#define WC          1.00000000000       // Wheel correction (Difference between the (referent) right and left wheel)
#define ER          2048                // Encoder resolution
#define ADW         230.35              // Axial distance between wheels

//#define _PI_        3.14159265359         // PI
#define _PI_        3.14159265358979323846
//#define _PI_        3.141592653589793238462643383279502884L
//#define _2_PI_      6.28318530718         // 2*PI
#define _2_PI_      6.28318530717958647692  // 2*PI

#define TICK_BY_MM  15.8612809466       // (ER/(_PI_*WD))
#define AWD_TICK    3653.64606604       // (ADW * TICK_BY_MM)


typedef struct robot
{
    float32_t X;
    float32_t Y;
    float32_t O;
    float32_t L;

    int32_t AngularVelocity;
    int32_t LinearVelocity;

    int32_t LeftVelocity;
    int32_t RigthVelocity;

    float32_t Xmm;
    float32_t Ymm;
    float32_t Odeg;
    float32_t Lmm;
} robot_t;


/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Location
{
    /**
	 * @class PID
	 * @brief Provides a singletron Odometry class
	 *
	 * HOWTO :
	 * - Get Odometry instance with Location::Odometry::GetInstance()
	 * - On init, Call Init() to init values
     * - Call periodically Compute() to compute the robot location
	 * - Call GetRobot() to get current location
	 */
class Odometry
{
public:

    /**
    * @brief Get instance method
    * @return Odometry
    *  instance
    */
    static Odometry* GetInstance ();

    /**
     * @brief Init coordinates
     * @param X : X cartesian coordinate (X plane)
     * @param Y : Y cartesian coordinate (Y plane)
     * @param O : O polar coordinate (pole)
     * @param L : L polar coordinate (axis)
     */
     Init(float32_t X, float32_t Y, float32_t O, float32_t L);

    /**
     * @brief Get current location
     * @param r : robot struct
     */
     GetRobot(robot_t * r);

     /**
      * @brief Set coordinate X and O during stall
      * @param X : X cartesian coordinate (X plane)
      * @param O : O polar coordinate (pole)
      */
    setXO(float32_t X, float32_t O);

    /**
     * @brief Set coordinate Y and O during stall
     * @param Y : Y cartesian coordinate (Y plane)
     * @param O : O polar coordinate (pole)
     */
    setYO(float32_t Y, float32_t O);

    /**
     * @brief Compute robot location (Should be called periodically)
     */
     Compute();

protected:
    /**
     * @brief Odometry default constructor
     */
    Odometry();

    /**
     * @brief Odometry constructor
     * @param X : X cartesian coordinate (X plane)
     * @param Y : Y cartesian coordinate (Y plane)
     * @param O : O polar coordinate (pole)
     * @param L : L polar coordinate (axis)
     */
    Odometry(float32_t X, float32_t Y, float32_t O, float32_t L);

    /**
     * @brief Odometry default destructor
     */
    ~Odometry();

    robot_t robot;

    HAL::Encoder* leftEncoder;
    HAL::Encoder* rightEncoder;

};


#endif /* ODOMETRY_H_ */
