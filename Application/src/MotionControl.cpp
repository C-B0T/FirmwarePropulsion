/**
 * @file    MotionControl.cpp
 * @author  Jeremy ROULLAND
 * @date    17 apr. 2017
 * @brief   Motion Control class
 */

#include "MotionControl.hpp"

namespace MotionControl
{

    BlMotionControl::BlMotionControl()
    {
        // Odometry instance created in standalone mode
        this->odometry = Odometry::GetInstance(true);

        // VC, PC instances creations
        this->vc = VelocityControl::GetInstance(true);
        this->pc = PositionControl::GetInstance(true);
        this->pg = ProfileGenerator::GetInstance(true);
        this->tp = TrajectoryPlanning::GetInstance(true);
    }

    void BlMotionControl::Update()
    {
        //this->tp->Compute();
        //this->pg->Compute();
        //this->pc->Compute();
        //this->vc->Compute();
    }


    SbSMotionControl::SbSMotionControl()
    {

    }

}
