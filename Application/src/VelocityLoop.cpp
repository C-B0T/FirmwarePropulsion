/**
 * @file	VelocityLoop.cpp
 * @author	Romain DESILLE
 * @date	10 déc. 2016
 * @brief	VelocityLoop class
 *
 *
 */

#include "VelocityLoop.hpp"
#include "common.h"

using namespace HAL;
using namespace Utils;
using namespace Location;
using namespace MotionControl;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_VELOCITY_PID_KP		(1.0f)
#define ANGULAR_VELOCITY_PID_KI		(0.0f)
#define ANGULAR_VELOCITY_PID_KD		(0.0f)

#define LINEAR_VELOCITY_PID_KP		(1.0f)
#define LINEAR_VELOCITY_PID_KI		(0.0f)
#define LINEAR_VELOCITY_PID_KD		(0.0f)


#define VL_TASK_STACK_SIZE		(64u)
#define VL_TASK_PRIORITY		(configMAX_PRIORITIES-1)	// Highest priority task

#define VL_TASK_EVENT_TIMER	(1u << 0u)

#define VL_TASK_PERIOD_MS	(5u) 							/**< 5ms velocity control loop */

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static VelocityLoop* _velocityLoop = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
	VelocityLoop* VelocityLoop::GetInstance(void)
	{
		if(_velocityLoop != NULL)
		{
			return _velocityLoop;
		}
		else
		{
			_velocityLoop = new VelocityLoop();

			return _velocityLoop;
		}
	}

	VelocityLoop::VelocityLoop(void)
	{
		this->name = "VelocityLoop";

		// Init PID
		this->angularVelocityPID = PID(ANGULAR_VELOCITY_PID_KP,
									   ANGULAR_VELOCITY_PID_KI,
									   ANGULAR_VELOCITY_PID_KD,
									   VL_TASK_PERIOD_MS);

		this->linearVelocityPID  = PID(LINEAR_VELOCITY_PID_KP,
									   LINEAR_VELOCITY_PID_KI,
									   LINEAR_VELOCITY_PID_KD,
									   VL_TASK_PERIOD_MS);

		// Init odometry
		this->odometry = Odometry::GetInstance();

		// Create task
		xTaskCreate((TaskFunction_t)(&VelocityLoop::taskHandler),
					this->name.c_str(),
					VL_TASK_STACK_SIZE,
					(void*)this,
					VL_TASK_PRIORITY,
					&this->taskHandle);

		// Create Timer
		this->loopTimer = xTimerCreate(this->name.c_str(),
									   pdMS_TO_TICKS(VL_TASK_PERIOD_MS),
									   pdTRUE,
									   (void*)this,
									   (TimerCallbackFunction_t)(&VelocityLoop::timerCallback));
	}

	void VelocityLoop::taskHandler(void* obj)
	{
/*		BrushlessMotor* instance = (BrushlessMotor*)obj;
		uint32_t event = 0;
		TickType_t prevTick = 0u,  tick = 0u;
		int32_t encoderPos = 0;
		float32_t period = 0.0f, speedFeedback = 0.0f, speed = 0.0f;

		// 1. Start loop timer
		xTimerStart(instance->loopTimer, 0u);

		// 2. Get tick count
		prevTick = xTaskGetTickCount();

		while(1)
		{
			// 2. Wait until timer elapsed
			if(xTaskNotifyWait(0,
							   0xFFFFFFFF,
							   &event,
							   portMAX_DELAY) == pdTRUE)
			{
				if(event == MOT_TASK_EVENT_TIMER)
				{
					// 3. Get tick
					tick = xTaskGetTickCount();

					period = (float32_t)tick - (float32_t)prevTick;

					//4. Get encoder value
					encoderPos = instance->encoder->GetRelativeValue();

					// 5. Get speed in encoder_tick/os_tick
					speedFeedback = (float32_t)((float32_t)encoderPos / period);

					// 6. Compute PID from speed feedback
					speed = instance->pid.Get(speedFeedback, period);

					// 7. Set speed
					instance->SetSpeed(speed);

					// 8. Set previous tick
					prevTick = tick;
				}
			}
		}*/
	}

	void VelocityLoop::timerCallback(TimerHandle_t handle)
	{
		VelocityLoop* instance = (VelocityLoop*)pvTimerGetTimerID(handle);

		xTaskNotify(instance->taskHandle,
					VL_TASK_EVENT_TIMER,
					eSetValueWithOverwrite);
	}
}
