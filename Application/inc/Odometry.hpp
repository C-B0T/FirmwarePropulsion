/**
 * @file	Odometry.hpp
 * @author	Romain DESILLE
 * @date	7 déc. 2016
 * @brief	Odometry class
 */

#ifndef INC_ODOMETRY_HPP_
#define INC_ODOMETRY_HPP_

#include "HAL.hpp"
#include "Utils.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
    int32_t LeftVelocity;
    int32_t RigthVelocity;

    float32_t AngularVelocity;
    float32_t LinearVelocity;

    float32_t X;
    float32_t Y;
    float32_t O;
    float32_t L;

    float32_t Xmm;
    float32_t Ymm;
    float32_t Odeg;

}ODOMETRY_DATA;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Odometry
 */
namespace Location
{
	/**
	 * @class Odometry
	 * @brief Odometry class
	 *
	 * Robot motion information calculated from encoders
	 */
	class Odometry
	{
	public:
		/**
		 * @brief Get instance method
		 * @return Odometry instance
		 */
		static Odometry* GetInstance (void);

		/**
		 * @private
		 * @brief compute the odometry
		 * @param obj : Always NULL
		 */
		void Compute (void);

		/**
		 * @brief Return instance name
		 */
		std::string Name()
		{
			return this->name;
		}

		ODOMETRY_DATA Data;

	private:

		/**
		 * @private
		 * @brief Private constructor
		 */
		Odometry (void);

		/**
		 * @private
		 * @brief Instance name
		 */
		std::string name;

		/**
		 * @private
		 * @brief left wheel encoder
		 */
		HAL::Encoder* leftEncoder;

		/**
		 * @private
		 * @brief right wheel encoder
		 */
		HAL::Encoder* rightEncoder;

	};
}

#endif /* INC_ODOMETRY_HPP_ */
