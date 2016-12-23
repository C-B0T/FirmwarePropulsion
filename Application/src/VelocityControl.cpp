/**
 * @file	VelocityControl.cpp
 * @author	Jeremy ROULLAND
 * @date	20 dec. 2016
 * @brief	VelocityControl class
 *
 *
 */

#include "VelocityControl.hpp"

using namespace HAL;
using namespace Utils;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_VELOCITY_PID_KP     (1.0f)
#define ANGULAR_VELOCITY_PID_KI     (0.0f)
#define ANGULAR_VELOCITY_PID_KD     (0.0f)

#define LINEAR_VELOCITY_PID_KP      (1.0f)
#define LINEAR_VELOCITY_PID_KI      (0.0f)
#define LINEAR_VELOCITY_PID_KD      (0.0f)

#define VL_TASK_PERIOD_MS           (5u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static VelocityControl* _velocityControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
    VelocityControl* VelocityControl::GetInstance(void)
    {
        // If VelocityControl instance already exists
        if(_velocityControl != NULL)
        {
            return _velocityControl;
        }
        else
        {
            _velocityControl = new VelocityControl();
        }
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
     * @brief  VelocityControl constructor
     */
    VelocityControl::VelocityControl()
    {
        this->name = "VelocityControl";

        this->angularVelocityPID = PID(ANGULAR_VELOCITY_PID_KP,
           ANGULAR_VELOCITY_PID_KI,
           ANGULAR_VELOCITY_PID_KD,
           VL_TASK_PERIOD_MS);

        this->linearVelocityPID  = PID(LINEAR_VELOCITY_PID_KP,
           LINEAR_VELOCITY_PID_KI,
           LINEAR_VELOCITY_PID_KD,
           VL_TASK_PERIOD_MS);

        this->odometry = Odometry::GetInstance();

        this->leftMotor = BrushlessMotorDriver::GetInstance(BrushlessMotorDriver::LEFT_MOTOR);
        this->rightMotor = BrushlessMotorDriver::GetInstance(BrushlessMotorDriver::RIGHT_MOTOR);

        this->angularVelocity = 0.0;
        this->linearVelocity  = 0.0;
    }

    /**
     * @brief  VelocityControl compute
     */
    VelocityControl::Compute()
    {

    }
}
