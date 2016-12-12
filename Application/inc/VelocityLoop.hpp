/**
 * @file	VelocityLoop.hpp
 * @author	Romain DESILLE
 * @date	10 déc. 2016
 * @brief	VelocityLoop class
 */

#ifndef INC_VELOCITYLOOP_HPP_
#define INC_VELOCITYLOOP_HPP_

#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

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
	 * @class VelocityLoop
	 * @brief Velocity loop class
	 *
	 * Handle velocity loop
	 */
	class VelocityLoop
	{
	public:
		/**
		 * @brief Get instance method
		 * @return VelocityLoop instance
		 */
		static VelocityLoop* GetInstance (void);

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



	private:

		/**
		 * @private
		 * @brief Private constructor
		 */
		VelocityLoop (void);

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

		/**
		 * @private
		 * @brief OS Task handle
		 *
		 * Used by speed control loop
		 */
		TaskHandle_t taskHandle;

		/**
		 * @private
		 * @brief OS Timer Handle
		 *
		 * Used by speed control loop
		 */
		TimerHandle_t loopTimer;

		/**
		 * @private
		 * @brief Speed control loop task handler
		 * @param obj : Always NULL
		 */
		void taskHandler (void* obj);

		/**
		 * @private
		 * @brief Instance timer callback
		 * @param handle : OS Timer handle
		 */
		void timerCallback (TimerHandle_t handle);
	};
}

#endif /* INC_VELOCITYLOOP_HPP_ */
