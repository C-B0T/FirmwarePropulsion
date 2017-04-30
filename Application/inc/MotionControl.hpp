/**
 * @file	MotionControl.hpp
 * @author	Jeremy ROULLAND
 * @date	3 jan. 2017
 * @brief	MotionControl namespace include file
 */

#ifndef INC_MOTIONCONTROL_HPP_
#define INC_MOTIONCONTROL_HPP_

/**
 * @namespace MotionControl
 * @brief motion controller objects
 *
 * Contains class such as PositionControl, VelocityControl, MotionProfile, etc.
 */

#include "common.h"

// For std::string
#include <algorithm>


#include "Odometry.hpp"
#include "VelocityControl.hpp"
#include "PositionControl.hpp"
#include "ProfileGenerator.hpp"
#include "TrajectoryPlanning.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

using namespace Location;


namespace MotionControl
{

    /**
     * @class FBMotionControl
     * @brief Feedback Controller class
     *
     * HOWTO :
     * -
     *
     */
    class FBMotionControl
    {
    public:
        /**
         * @brief Get instance method
         * @return VelocityLoop instance
         */
        static FBMotionControl* GetInstance();

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        uint16_t GetStatus()
        {
        	return this->status;
        }

        void Enable();

        void Disable();

        void DisableSafeguard()
        {
        	this->safeguard = true;
        }
        void EnableSafeguard()
        {
        	this->safeguard = false;
        }
        void ToggleSafeguard()
        {
        	this->safeguard = !this->safeguard;
        }
        bool GetSafeguard()
        {
        	return this->safeguard;
        }

    protected:
        FBMotionControl();

        // 16 Flags Status
        uint16_t status;

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        Odometry           *odometry;
        VelocityControl    *vc;
        PositionControl    *pc;
        ProfileGenerator   *pg;
        TrajectoryPlanning *tp;


        bool enable;

        bool safeguard;

        /**
         * @brief Compute motion control
         */
        void Compute(float32_t period);

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by speed control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Speed control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };

}

#endif /* INC_MOTIONCONTROL_HPP_ */
