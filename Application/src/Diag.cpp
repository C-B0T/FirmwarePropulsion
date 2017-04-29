/**
 * @file    Diag.cpp
 * @author  Jeremy ROULLAND
 * @date    29 apr. 2017
 * @brief   Diag class
 */

#include "Diag.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define DIAG_TASK_STACK_SIZE          (256u)
#define DIAG_TASK_PRIORITY            (2u)

#define DIAG_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Diag* _diag = NULL;

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

Diag* Diag::GetInstance()
{
    // If VelocityControl instance already exists
    if(_diag != NULL)
    {
        return _diag;
    }
    else
    {
        _diag = new Diag();
        return _diag;
    }
}

Diag::Diag()
{
    this->name = "Diag";
    this->taskHandle = NULL;

    this->enable = false;

    // Create task
    xTaskCreate((TaskFunction_t)(&Diag::taskHandler),
                this->name.c_str(),
                DIAG_TASK_STACK_SIZE,
                NULL,
                DIAG_TASK_PRIORITY,
                NULL);

    this->odometry = Odometry::GetInstance();
    this->vc = VelocityControl::GetInstance();
    this->pc = PositionControl::GetInstance();
    this->pg = ProfileGenerator::GetInstance();
    this->tp = TrajectoryPlanning::GetInstance();
    this->mc = FBMotionControl::GetInstance();

}

void Diag::Traces()
{
    
}

void Diag::Led()
{

}

void Diag::Compute(float32_t period)
{
	this->Led();

	if(this->enable)
		this->Traces();
}

void Diag::taskHandler (void* obj)
{
    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(DIAG_TASK_PERIOD_MS);

    Diag* instance = _diag;
    TickType_t prevTick = 0u,  tick = 0u;

    float32_t period = 0.0f;

    // 1. Initialise periodical task
    xLastWakeTime = xTaskGetTickCount();

    // 2. Get tick count
    prevTick = xTaskGetTickCount();

    while(1)
    {
        // 2. Wait until period elapse
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 3. Get tick
        tick = xTaskGetTickCount();

        period = static_cast<float32_t>(tick) -
                 static_cast<float32_t>(prevTick);

        //4. Compute Diag informations
        instance->Compute(period);

        // 5. Set previous tick
        prevTick = tick;
    }
}
