/**
 * @file	VelocityControl.hpp
 * @author	Jeremy ROULLAND
 * @date	20 dec. 2016
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
         * @brief Velocity Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            VELOCITY_MAX    //!< VELOCITY_MAX
        };

        /**
         * @brief Get instance method
         * @return VelocityLoop instance
         * @return BrushlessMotor instance
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
         * @brief Compute robot velocity
         */
        Compute();

    protected:

        /**
         * @protected
         * @brief Private constructor
         */
        VelocityControl ();

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        /**
         * @protected
         * @brief Angular velocity PID controller
         */
        Utils::PID	angularVelocityPID;

        /**
         * @protected
         * @brief Linear velocity PID controller
         */
        Utils::PID	linearVelocityPID;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief Left motor brushless driver instance
         */
        HAL::BrushlessMotorDriver* leftMotor;

        /**
         * @protected
         * @brief Right motor brushless driver instance
         */
        HAL::BrushlessMotorDriver* rightMotor;

        /**
         * @protected
         * @brief Angular velocity setpoint
         */
        float32_t angularVelocity;

        /**
         * @protected
         * @brief Linear velocity setpoint
         */
        float32_t linearVelocity;

    };
}

#endif /* INC_VELOCITYCONTROL_HPP_ */
