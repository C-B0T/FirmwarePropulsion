/**
 * @file	VelocityControl.cpp
 * @author	Jeremy ROULLAND
 * @date	20 dec. 2016
 * @brief	VelocityControl class
 *
 *
 */

#include "VelocityControl.hpp"
#include "common.h"

#include <stdio.h>

using namespace HAL;
using namespace Utils;
using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_VELOCITY_PID_KP     (0.06f)
//#define ANGULAR_VELOCITY_PID_KI     (0.00001f)
#define ANGULAR_VELOCITY_PID_KI     (0.0f)
#define ANGULAR_VELOCITY_PID_KD     (0.0f)

//#define LINEAR_VELOCITY_PID_KP      (0.002f)
//#define LINEAR_VELOCITY_PID_KI      (0.00001f)
#define LINEAR_VELOCITY_PID_KP      (0.0f)
#define LINEAR_VELOCITY_PID_KI      (0.0f)
#define LINEAR_VELOCITY_PID_KD      (0.0f)

#define VC_MOTOR_LEFT				(BrushlessMotorDriver::ID::DRIVER0)
#define VC_MOTOR_RIGHT				(BrushlessMotorDriver::ID::DRIVER1)

#define VC_TASK_STACK_SIZE			(256u)
#define VC_TASK_PRIORITY			(configMAX_PRIORITIES-3)	// Highest priority task

#define VC_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::VelocityControl* _velocityControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static VC_DEF _getDefStructure (enum VelocityControl::ID id)
{
	VC_DEF def;

	assert(id < VelocityControl::VELOCITY_MAX);

	switch(id)
	{
		case VelocityControl::ANGULAR:
			def.Motors.ID_left		=	VC_MOTOR_LEFT;
			def.Motors.ID_right		=	VC_MOTOR_RIGHT;
			def.PID_Angular.kp		=	ANGULAR_VELOCITY_PID_KP;
			def.PID_Angular.ki		=	ANGULAR_VELOCITY_PID_KI;
			def.PID_Angular.kd		=	ANGULAR_VELOCITY_PID_KD;
			break;

		case VelocityControl::LINEAR:
			def.Motors.ID_left		=	VC_MOTOR_LEFT;
			def.Motors.ID_right		=	VC_MOTOR_RIGHT;
			def.PID_Linear.kp		=	LINEAR_VELOCITY_PID_KP;
			def.PID_Linear.ki		=	LINEAR_VELOCITY_PID_KI;
			def.PID_Linear.kd		=	LINEAR_VELOCITY_PID_KD;
			break;

		default:
			break;
	}

	return def;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
    VelocityControl* VelocityControl::GetInstance()
    {
        // If VelocityControl instance already exists
        if(_velocityControl != NULL)
        {
            return _velocityControl;
        }
        else
        {
            _velocityControl = new VelocityControl();
            return _velocityControl;
        }
    }

    /**
     * @brief  VelocityControl constructor
     */
    VelocityControl::VelocityControl()
    {
		this->name = "VelocityControl";

		// Init Angular velocity control
		this->def = _getDefStructure(VelocityControl::ANGULAR);
		this->pid_angular = PID(this->def.PID_Angular.kp,
								this->def.PID_Angular.ki,
								this->def.PID_Angular.kd,
								VC_TASK_PERIOD_MS);

		// Init Linear velocity control
		this->def = _getDefStructure(VelocityControl::LINEAR);
		this->pid_linear  = PID(this->def.PID_Linear.kp,
								this->def.PID_Linear.ki,
								this->def.PID_Linear.kd,
								VC_TASK_PERIOD_MS);


        this->odometry = Odometry::GetInstance();

        this->leftMotor  = BrushlessMotorDriver::GetInstance(this->def.Motors.ID_left);
        this->rightMotor = BrushlessMotorDriver::GetInstance(this->def.Motors.ID_right);

        this->angularVelocity = 0.0f;
        this->linearVelocity  = 0.0f;
    }

    /**
     * @brief  VelocityControl compute
     */
    void VelocityControl::Compute(float32_t period)
    {
    	float32_t currentAngularVelocity = 0.0;
    	float32_t currentLinearVelocity  = 0.0;

    	float32_t speed_angular = 0.0;
    	float32_t speed_linear  = 0.0;
    	float32_t speed_left  = 0.0;
    	float32_t speed_right = 0.0;

    	// Get current velocities
    	currentAngularVelocity = odometry->GetAngularVelocity(period);
    	currentLinearVelocity  = odometry->GetLinearVelocity(period);

		// Set setpoint velocities
    	this->pid_angular.SetSetpoint(this->angularVelocity);
    	this->pid_linear.SetSetpoint(this->linearVelocity);

    	// Compute PIDs
    	speed_angular = this->pid_angular.Get(currentAngularVelocity, period);
    	speed_linear  = this->pid_linear.Get(currentLinearVelocity, period);

    	// Angular/Linear to Left/Right
    	speed_left  = speed_linear - speed_angular;
    	speed_right = speed_linear + speed_angular;

    	// Set speed to Motors
    	this->leftSpeed  = -speed_left;
		this->rightSpeed =  speed_right;

    	leftMotor->SetMotorSpeed(this->leftSpeed);
    	rightMotor->SetMotorSpeed(this->rightSpeed);

    }
}
