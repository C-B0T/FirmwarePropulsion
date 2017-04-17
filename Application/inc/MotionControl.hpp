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

#include "Odometry.hpp"
#include "VelocityControl.hpp"
#include "PositionControl.hpp"
#include "ProfileGenerator.hpp"

#include "TrajectoryPlanning.hpp"

using namespace Location;

namespace MotionControl
{
    /**
     * @class BlMotionControl
     * @brief Brushless Motion Controller class
     *
     * HOWTO :
     * -
     *
     */
    class BlMotionControl
    {
    public:
        BlMotionControl();
        void Update();

    protected:
        Odometry           *odometry;
        VelocityControl    *vc;
        PositionControl    *pc;
        ProfileGenerator   *pg;
        TrajectoryPlanning *tp;

    };

    /**
     * @class SbSMotionControl
     * @brief Step by Step Motion Controller class
     *
     * HOWTO :
     * -
     *
     */
    class SbSMotionControl
    {
    public:
        SbSMotionControl();

    private:

    };
}

#endif /* INC_MOTIONCONTROL_HPP_ */
