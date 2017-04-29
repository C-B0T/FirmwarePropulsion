/**
 * @file    MotionControl.cpp
 * @author  Jeremy ROULLAND
 * @date    17 apr. 2017
 * @brief   Motion Control class
 */

#include "MotionControl.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MC_TASK_STACK_SIZE          (256u)
#define MC_TASK_PRIORITY            (configMAX_PRIORITIES-3)

#define MC_TASK_PERIOD_MS           (5u)
#define TP_TASK_PERIOD_MS           (100u)
#define PG_TASK_PERIOD_MS           (10u)
#define PC_TASK_PERIOD_MS           (10u)
#define VC_TASK_PERIOD_MS           (5u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::FBMotionControl* _motionControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{

    FBMotionControl* FBMotionControl::GetInstance()
    {
        // If VelocityControl instance already exists
        if(_motionControl != NULL)
        {
            return _motionControl;
        }
        else
        {
            _motionControl = new FBMotionControl();
            return _motionControl;
        }
    }

    FBMotionControl::FBMotionControl()
    {
        this->name = "MotionControl";
        this->taskHandle = NULL;

        // Odometry instance created in standalone mode
        this->odometry = Odometry::GetInstance(true);

        // VC, PC instances creations
        this->vc = VelocityControl::GetInstance(false);
        this->pc = PositionControl::GetInstance(false);
        this->pg = ProfileGenerator::GetInstance(false);
        this->tp = TrajectoryPlanning::GetInstance(false);

        // Create task
        xTaskCreate((TaskFunction_t)(&FBMotionControl::taskHandler),
                    this->name.c_str(),
                    MC_TASK_STACK_SIZE,
                    NULL,
                    MC_TASK_PRIORITY,
                    NULL);
    }

    void FBMotionControl::Compute(float32_t period)
    {
        static uint32_t localTime = 0;

        localTime += MC_TASK_PERIOD_MS;

        if( (localTime % TP_TASK_PERIOD_MS) == 0)
            this->tp->Compute((period * TP_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);
        if( (localTime % PG_TASK_PERIOD_MS) == 0)
            this->pg->Compute((period * PG_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);
        if( (localTime % PC_TASK_PERIOD_MS) == 0)
            this->pc->Compute((period * PC_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);
        if( (localTime % VC_TASK_PERIOD_MS) == 0)
            this->vc->Compute((period * VC_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);

    }


    void FBMotionControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(MC_TASK_PERIOD_MS);

        FBMotionControl* instance = _motionControl;
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

            //4. Compute velocity (VelocityControl)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }
}
