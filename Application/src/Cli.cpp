/**
 * @file    CLI.cpp
 * @author  Jeremy ROULLAND
 * @date    23 mar. 2017
 * @brief   Command Line Interface class
 */

#include "Cli.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CLI_BUF_MAX        40

#define CLI_TASK_STACK_SIZE          (256u)
#define CLI_TASK_PRIORITY            (1u)

#define CLI_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static CLI* _cli = NULL;

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

CLI* CLI::GetInstance()
{
    // If VelocityControl instance already exists
    if(_cli != NULL)
    {
        return _cli;
    }
    else
    {
        _cli = new CLI();
        return _cli;
    }
}

CLI::CLI()
{
    this->name = "Cli";
    this->taskHandle = NULL;

    // Create task
    xTaskCreate((TaskFunction_t)(&CLI::taskHandler),
                this->name.c_str(),
                CLI_TASK_STACK_SIZE,
                NULL,
                CLI_TASK_PRIORITY,
                NULL);

    this->odometry = Odometry::GetInstance();
    this->vc = VelocityControl::GetInstance();
    this->pc = PositionControl::GetInstance();
    this->pg = ProfileGenerator::GetInstance();
    this->tp = TrajectoryPlanning::GetInstance();
    this->mc = FBMotionControl::GetInstance();

    this->diag = Diag::GetInstance();
}


void CLI::Start()
{
	putchar('>');
}

void CLI::Compute(float32_t period)
{
    static char buf[CLI_BUF_MAX] = {0};
    static uint16_t i = 0;

    char c;
    char *pch;

    c = getchar();

    if(c == '&')
    {
        vc->Stop();
    	printf("!!EMERGENCY STOP!!");

    }
    else if(c == '(')
    {
        this->diag->Toggle();
    }
    else if(c == '_')
    {
    	putchar(c);
    }
    else if(c == ')')
    {
    	putchar(c);
    }
    else if(c == '=')
    {
    	putchar(c);
    }
    else if(c == ',')
    {
    	putchar(c);
    }
    else if(c == ';')
    {
    	putchar(c);
    }
    else if(c == ':')
    {
    	putchar(c);
    }
    else if(c == '!')
    {
    	putchar(c);
    }
    else if( ((c >= '0') && (c <= '9')) ||
        ((c >= 'a') && (c <= 'z')) ||
        ((c >= 'A') && (c <= 'Z')) ||
         (c == '.') || (c == ' ')  ||
         (c == '-')  )
    {
        if(i < (CLI_BUF_MAX-2))
        {
            putchar(c);
            buf[i] = c;
            i++;
            buf[i] = '\0';
        }

    }
    else if(c == '\b')
    {
        putchar('\b');
        putchar(' ');
        putchar('\b');
        if(i>0)
            i--;
        buf[i] = '\0';
    }
    else if( (c == '\r') || (c == '\n') )
    {
        pch = strtok (buf," ");

        if(strcmp(pch,"help") == 0)
        {
            printf("\r\n## help (v0.1):\r\n");
            printf(" Shortcut:\r\n");
            printf(" - &            \tEmergency stop\r\n");
            printf(" - (            \tToggle traces\r\n");
            printf(" Command:\r\n");
            printf(" - golin <l>    \tGo Linear\r\n");
            printf(" - goang <a>    \tGo Angular\r\n");
            printf(" - goto <x> <y> \tGo to X,Y\r\n");
            //printf(" - stop <%%>     \tStop %% Brake\r\n");
        }
        else if(strcmp(pch,"golin") == 0)
        {
            float l;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                l = strtof(pch, NULL);
            else
                l = 0.0;

            printf("\r\ngolin %.3f", l);
            tp->goLinear(l);
        }
        else if(strcmp(pch,"goang") == 0)
        {
            float a;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                a = strtof(pch, NULL);
            else
                a = 0.0;

            printf("\r\ngoang %.3f", a);
            tp->goAngular(a);
        }
        else if(strcmp(pch,"goto") == 0)
        {
            float x, y;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                x = strtof(pch, NULL);
            else
                x = 0.0;
            if(pch != NULL)
                y = strtof(pch, NULL);
            else
                y = 0.0;

            printf("\r\ngoto %.3f %.3f", x, y);
            tp->gotoXY(x,y);
        }
        else if(strcmp(pch,"stop") == 0)
        {
            float bk = 1.0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                bk = strtof(pch, NULL);
            printf("\r\nstop (%.1f Brake)", bk);
            tp->stop();
            //vc->Brake(bk);	/* Brake is overwritten somewhere */
        }
        else if(strcmp(pch,"checkup") == 0)
        {
            // TODO: Checkup
            printf("\r\ncheckup");
        }
        else if(strcmp(pch,"safeguard") == 0)
        {
            tp->ToggleSafeguard();
            printf("\r\nsafeguard=%u", tp->GetSafeguard());
        }
        else
        {
        	if(i > 0)
        		printf("\r\nBad cmd!!");
        }

        i=0;
        buf[i] = '\0';
        putchar('\r');
        putchar('\n');
        putchar('>');
    } /*else if( (c == '\r') || (c == '\n') )*/

} /* void CLI::Compute() */

void CLI::taskHandler (void* obj)
{
    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(CLI_TASK_PERIOD_MS);

    CLI* instance = _cli;
    TickType_t prevTick = 0u,  tick = 0u;

    float32_t period = 0.0f;

    // 0. Delay
    vTaskDelay(pdMS_TO_TICKS(3000u));

    instance->Start();

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
