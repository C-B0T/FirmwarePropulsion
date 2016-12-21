/**
 * @file	VelocityControl.hpp
 * @author	Jeremy ROULLAND
 * @date	20 déc. 2016
 * @brief	VelocityControl class
 */

#ifndef INC_VELOCITYCONTROL_HPP_
#define INC_VELOCITYCONTROL_HPP_

#include "common.h"
#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class VelocityControl
     * @brief Velocity control class
     *
     * HOWTO :
     * -
     *
     */
    class VelocityControl
    {
    public:
    /**
     * @brief Get instance method
     * @return VelocityLoop instance
     */
    static VelocityControl* GetInstance ();

    /**
     * @brief Return instance name
     */
    std::string Name()
    {
    return this->name;
    }

    /**
     * @brief Set the angular velocity
     */
    void SetAngularVelocity(float32_t angularVelocity)
    {
    this->angularVelocity = angularVelocity;
    }

    /**
     * @brief Set the linear velocity
     */
    void SetLinearVelocity(float32_t linearVelocity)
    {
    this->linearVelocity = linearVelocity;
    }

    /**
     * @brief Compute robot velocity (Should be called periodically)
     */
    Compute();

    private:

    /**
     * @private
     * @brief Private constructor
     */
    VelocityControl (void);

    /**
     * @private
     * @brief Instance name
     */
    std::string name;

    /**
     * @private
     * @brief Angular velocity PID controller
     */
    Utils::PID	angularVelocityPID;

    /**
     * @private
     * @brief Linear velocity PID controller
     */
    Utils::PID	linearVelocityPID;

    /**
     * @private
     * @brief Odometry instance
     */
    Location::Odometry* odometry;

    /**
     * @private
     * @brief Left motor brushless driver instance
     */
    HAL::BrushlessMotorDriver* leftMotor;

    /**
     * @private
     * @brief Right motor brushless driver instance
     */
    HAL::BrushlessMotorDriver* rightMotor;

    /**
     * @private
     * @brief Angular velocity setpoint
     */
    float32_t angularVelocity;

    /**
     * @private
     * @brief Linear velocity setpoint
     */
    float32_t linearVelocity;


    };
}

#endif /* INC_VELOCITYCONTROL_HPP_ */
