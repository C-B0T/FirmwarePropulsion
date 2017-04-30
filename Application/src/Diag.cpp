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

    this->led1 = GPIO::GetInstance(GPIO::GPIO6);
    this->led2 = GPIO::GetInstance(GPIO::GPIO7);
    this->led3 = GPIO::GetInstance(GPIO::GPIO8);
    this->led4 = GPIO::GetInstance(GPIO::GPIO9);

}

void Diag::Traces()
{
    robot_t r;

    this->odometry->GetRobot(&r);

    //printf("%.3f\t%.3f\t%.3f\r\n", r.X, r.Y, r.O);
    //printf("%ld\t%ld\t%.1f\r\n", r.Xmm, r.Ymm, r.Odeg);

    printf("%ld\t%ld\t%.3f\t%.3f\t%.3f\t%.3f\r\n", tp->GetStep(), pg->GetLinearPhase(), pg->GetLinearPositionProfiled(), pc->GetLinearPosition(), vc->GetLinearVelocity(), vc->GetLinearSpeed());
}

void Diag::Led()
{
	static uint32_t localTime = 0;

	localTime += (uint32_t)DIAG_TASK_PERIOD_MS;

	// Led direction
	static uint32_t LedId = 4;
	switch(LedId)
	{
	case 1:
		this->led1->Toggle();
		break;
	case 2:
		this->led2->Toggle();
		break;
	case 3:
		this->led3->Toggle();
		break;
	case 4:
		this->led4->Toggle();
		break;
	default:
		break;
	}
	LedId--;
	if(LedId < 1)
		LedId = 4;

}

void Diag::Compute(float32_t period)
{
	static uint32_t localTime = 0;

	localTime += (uint32_t)DIAG_TASK_PERIOD_MS;


	if((localTime % 100) == 0)
	{
		this->Led();
	}

	if((localTime % 10) == 0)
	{
		if(this->enable)
			this->Traces();
	}
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
