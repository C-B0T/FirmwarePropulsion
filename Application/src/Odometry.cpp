/**
 * @file    Odometry.cpp
 * @author  Jeremy ROULLAND
 * @date    3 dec. 2016
 * @brief   Odometry is the use to estimate position
 */

#include "Odometry.h"

Odometry::Odometry()
{
    robot.X = 0.0;
    robot.Y = 0.0;
    robot.O = 0.0;
    robot.L = 0.0;

    robot.Xmm = 0.0;
    robot.Ymm = 0.0;
    robot.Omm = 0.0;
    robot.Lmm = 0.0;
}


Odometry::Odometry(float32_t X, float32_t Y, float32_t O, float32_t L)
{
    robot.X = X;
    robot.Y = Y;
    robot.O; = O;
    robot.L; = L;

    robot.Xmm = X*TICK_BY_MM;
    robot.Ymm = Y*TICK_BY_MM;
    robot.Odeg = O*TICK_BY_MM;
    robot.Lmm = L*TICK_BY_MM;
}


Odometry::~Odometry()
{

}


Odometry::getRobot(robot_t *r)
{
    r.X = robot.X;
    r.Y = robot.Y;
    r.O = robot.O;
    r.L = robot.L;

    r.AngularVelocity = robot.AngularVelocity;
    r.LinearVelocity = robot.LinearVelocity;
    
    r.LeftVelocity = robot.LeftVelocity;
    r.RigthVelocity = robot.RigthVelocity;

    r.Xmm = robot.Xmm;
    r.Ymm = robot.Ymm;
    r.Odeg = robot.Odeg;
    r.Lmm = robot.Lmm;
}


Odometry::setXY(float32_t X, float32_t Y)
{
    robot.X = X;
    robot.Y = Y;

    robot.Xmm = X*TICK_BY_MM;
    robot.Ymm = Y*TICK_BY_MM;
}


Odometry::setXO(float32_t X, float32_t O)
{
    robot.X = X;
    robot.O = O;

    robot.Xmm = X*TICK_BY_MM;
    robot.Odeg = (180.0* O) / _PI_;
}


Odometry::setYO(float32_t Y, float32_t O)
{
    robot.Y = Y;
    robot.O = O;

    robot.Ymm = Y*TICK_BY_MM;
    robot.Odeg = (180.0* O) / _PI_;
}


Odometry::compute()
{
    int16_t dL = 0;
    int16_t dR = 0;

    float16_t dX = 0.0;
    float16_t dY = 0.0;

    float16_t Vo = 0.0;
    float16_t Vl = 0.0;

    float16_t dLf = 0.0;
    float16_t dRf = 0.0;

    Encoders.getValues(&dL, &dR);
    
    dLf = static_cast<float16_t>(dL) * WC;
    dRf = static_cast<float16_t>(dR);

    Vo =  dRf - dLf;
    Vl = (dRf + dLf) / 2.0;

    robot.O += Vo/AWD_TICK;    
    robot.L += Vl;
    
    robot.AngularVelocity = static_cast<int16_t>(Vo);
    robot.LinearVelocity  = static_cast<int16_t>(Vl);

    while(robot.O > _2_PI_)
        robot.O -= _2_PI_;
    while(robot.O < -_2_PI_)
        robot.O += _2_PI_;

    dX = cosf(robot.O) * Vl;
    dY = sinf(robot.O) * Vl;

    robot.X += dX;
    robot.Y += dY;
    
    robot.LeftVelocity  = dL;
    robot.RigthVelocity = dR;
}


