/**
 * @file	PositionControl.cpp
 * @author	Jeremy ROULLAND
 * @date	25 dec. 2016
 * @brief	PositionControl class
 *
 *
 */

#include "PositionControl.hpp"
#include "common.h"

#include <stdio.h>

using namespace HAL;
using namespace Utils;
using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_POSITION_PID_KP     (0.0f)
#define ANGULAR_POSITION_PID_KI     (0.0f)
#define ANGULAR_POSITION_PID_KD     (0.0f)

#define LINEAR_POSITION_PID_KP      (0.0f)
#define LINEAR_POSITION_PID_KI      (0.0f)
#define LINEAR_POSITION_PID_KD      (0.0f)

#define PC_ANGULAR_VELOCITY			(VelocityControl::ID::ANGULAR)
#define PC_LINEAR_VELOCITY			(VelocityControl::ID::ANGULAR)

#define PC_TASK_STACK_SIZE			(256u)
#define PC_TASK_PRIORITY			(configMAX_PRIORITIES-4)	// Highest priority task

#define PC_TASK_PERIOD_MS           (20u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::PositionControl* _positionControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PC_DEF _getDefStructure (enum PositionControl::ID id)
{
	PC_DEF def;

	assert(id < PositionControl::POSITION_MAX);

	switch(id)
	{
		case PositionControl::ANGULAR:
			def.Velocity.ID_Angular	=	PC_ANGULAR_VELOCITY;
			def.Velocity.ID_Linear	=	PC_LINEAR_VELOCITY;
			def.PID_Angular.kp		=	ANGULAR_POSITION_PID_KP;
			def.PID_Angular.ki		=	ANGULAR_POSITION_PID_KI;
			def.PID_Angular.kd		=	ANGULAR_POSITION_PID_KD;
			break;

		case PositionControl::LINEAR:
			def.Velocity.ID_Angular	=	PC_ANGULAR_VELOCITY;
			def.Velocity.ID_Linear	=	PC_LINEAR_VELOCITY;
			def.PID_Linear.kp		=	LINEAR_POSITION_PID_KP;
			def.PID_Linear.ki		=	LINEAR_POSITION_PID_KI;
			def.PID_Linear.kd		=	LINEAR_POSITION_PID_KD;
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
    PositionControl* PositionControl::GetInstance()
    {
        // If PositionControl instance already exists
        if(_positionControl != NULL)
        {
            return _positionControl;
        }
        else
        {
            _positionControl = new PositionControl();
            return _positionControl;
        }
    }

    /**
     * @brief  PositionControl constructor
     */
    PositionControl::PositionControl()
    {
		this->name = "PositionControl";

		// Init Angular velocity control
		this->def = _getDefStructure(PositionControl::ANGULAR);
		this->pid_angular = PID(this->def.PID_Angular.kp,
								this->def.PID_Angular.ki,
								this->def.PID_Angular.kd,
								PC_TASK_PERIOD_MS);

		// Init Linear velocity control
		this->def = _getDefStructure(PositionControl::LINEAR);
		this->pid_linear  = PID(this->def.PID_Linear.kp,
								this->def.PID_Linear.ki,
								this->def.PID_Linear.kd,
								PC_TASK_PERIOD_MS);


        this->odometry = Odometry::GetInstance();

        this->velocityControl  = VelocityControl::GetInstance();

        this->angularPosition = 0.0f;
        this->linearPosition  = 0.0f;
    }

    /**
     * @brief  PositionControl compute
     */
    void PositionControl::Compute(float32_t period)
    {
		float32_t currentAngularPosition = 0.0;
    	float32_t currentLinearPosition  = 0.0;

		float32_t angularVelocity = 0.0;
    	float32_t linearVelocity  = 0.0;

    	this->pid_angular.SetSetpoint(this->angularPosition);
    	this->pid_linear.SetSetpoint(this->linearPosition);

    	angularVelocity = this->pid_angular.Get(odometry->GetAngularPosition(), period);
    	linearVelocity  = this->pid_linear.Get(odometry->GetLinearPosition(), period);

    	this->angularVelocity = angularVelocity;
		this->linearVelocity  = linearVelocity;

		velocityControl->SetAngularVelocity(this->angularVelocity);
		velocityControl->SetLinearVelocity(this->linearVelocity);

    }
}
