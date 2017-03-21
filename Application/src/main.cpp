/**
 * @file    main.cpp
 * @author  Kevin WYSOCKI
 * @date    8 nov. 2016
 * @brief   Main
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../STM32_Driver/inc/stm32f4xx.h"
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"

#include "HAL.hpp"
#include "BrushlessMotor.hpp"
#include "Odometry.hpp"
#include "VelocityControl.hpp"
#include "PositionControl.hpp"
#include "ProfileGenerator.hpp"
#include "TrajectoryPlanning.hpp"

#include "../../STM32_Driver/inc/stm32f4xx_it.h"

using namespace HAL;
using namespace Utils;
using namespace Location;
using namespace MotionControl;



extern "C" void hard_fault_handler_c(unsigned int * hardfault_args)
{
	  unsigned int stacked_r0;
	  unsigned int stacked_r1;
	  unsigned int stacked_r2;
	  unsigned int stacked_r3;
	  unsigned int stacked_r12;
	  unsigned int stacked_lr;
	  unsigned int stacked_pc;
	  unsigned int stacked_psr;

	  stacked_r0 = ((unsigned long) hardfault_args[0]);
	  stacked_r1 = ((unsigned long) hardfault_args[1]);
	  stacked_r2 = ((unsigned long) hardfault_args[2]);
	  stacked_r3 = ((unsigned long) hardfault_args[3]);

	  stacked_r12 = ((unsigned long) hardfault_args[4]);
	  stacked_lr = ((unsigned long) hardfault_args[5]);
	  stacked_pc = ((unsigned long) hardfault_args[6]);
	  stacked_psr = ((unsigned long) hardfault_args[7]);

	  printf ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
	  printf ("R0 = %x\r\n", stacked_r0);
	  printf ("R1 = %x\r\n", stacked_r1);
	  printf ("R2 = %x\r\n", stacked_r2);
	  printf ("R3 = %x\r\n", stacked_r3);
	  printf ("R12 = %x\r\n", stacked_r12);
	  printf ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
	  printf ("PC [R15] = %x  program counter\r\n", stacked_pc);
	  printf ("PSR = %x\r\n", stacked_psr);
	  printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
	  printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
	  printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
	  printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
	  printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
	  printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);

	  while (1);
}
void HardFault_Handler(void)
{
__ASM(".extern hard_fault_handler_c");
__ASM("TST LR, #4");
__ASM("ITE EQ");
__ASM("MRSEQ R0, MSP");
__ASM("MRSNE R0, PSP");
__ASM("B hard_fault_handler_c");
}

void BusFault_Handler(void)
{
	while(1);
}
void MemManage_Handler(void)
{
	while(1);
}
void WWDG_IRQHandler(void)
{
	while(1);
}
void UsageFault_Handler(void)
{
	while(1);
}


TaskHandle_t xHandleTraces;

/**
 * @brief Initialize hardware
 */
static void HardwareInit (void)
{
	// Ensure all priority bits are assigned as preemption priority bits.
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	// Enable all GPIO clock
	RCC_AHB1PeriphClockCmd((RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
			 	 	 	 	RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
							RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI),
						    ENABLE);

	// Enable Timer clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5,
						   ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,
						   ENABLE);

	// Enable USART Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

}

TrajectoryPlanning *pTp = NULL;

/**
 * @brief Main task handler
 */
void TASKHANDLER_Test (void * obj)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    //float32_t X[10] = {0.2,0.8,0.8,0.2,0.2}, Y[10] = {0.5,1.0,1.6,1.4,0.0};
    float32_t X[10] = {0.2,0.8,0.8,0.2,0.2}, Y[10] = {0.5,1.0,1.6,1.4,0.2};

    // Get instances
    GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);
    /*GPIO *led2 = GPIO::GetInstance(GPIO::GPIO7);
    GPIO *led3 = GPIO::GetInstance(GPIO::GPIO8);	// Crash with 3 leds
    GPIO *led4 = GPIO::GetInstance(GPIO::GPIO9);*/

    PositionControl *pc = PositionControl::GetInstance();
    //ProfileControl *pfc = ProfileControl::GetInstance();

    TrajectoryPlanning tp = TrajectoryPlanning();
    pTp = &tp;

    MotionProfile profile = MotionProfile(0.2,1.0);

    float32_t time = 0.0;

    led1->Toggle();

    // Wait 3sec before start
    vTaskDelay(pdMS_TO_TICKS(3000u));

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);


        /*led2->Toggle();
        led3->Toggle();
        led4->Toggle();*/

        // Set maximum
        /*pfc->SetAngularMaxVel(12.0);	// 12.0
        pfc->SetAngularMaxAcc(18.0);	// 18.0
        pfc->SetAngularProfile(MotionProfile::POLY5);
        pfc->SetLinearMaxVel(2.0);	// 2.0
        pfc->SetLinearMaxAcc(3.0);	// 3.0
        pfc->SetLinearProfile(MotionProfile::POLY5);	//POLY5
        */
        /*
        pc->SetAngularMaxVel(3.14);	// 12.0
        pc->SetAngularMaxAcc(3.14);	// 18.0
        pc->SetAngularProfile(MotionProfile::POLY5);
        pc->SetLinearMaxVel(0.5);	// 2.0
        pc->SetLinearMaxAcc(1.0);	// 3.0
        pc->SetLinearProfile(MotionProfile::POLY5);	//POLY5
        */

        /*
        pc->SetLinearPosition(2.0);
        while(pc->isPositioningFinished() == false) vTaskDelay(10);
        pc->Stop();
        */

        /*
        pc->SetAngularPosition(3.141592654/2.0);
        pc->SetLinearPosition(2.0);
        while(pc->isPositioningFinished() == false) vTaskDelay(10);
        pc->Stop();
        */

        /*time = ((float32_t)xTaskGetTickCount()) / 1000.0;
        profile.SetSetPoint(1.0, 0.0, time);*/

        while(1)
        {
        	// LinearPlan
            /*tp.gotoXY(X[0], Y[0]);
            do{vTaskDelay(100);}while(tp.update() == 0);
            tp.gotoXY(X[1], Y[1]);
            do{vTaskDelay(100);}while(tp.update() == 0);
            tp.gotoXY(X[2], Y[2]);
            do{vTaskDelay(100);}while(tp.update() == 0);
            tp.gotoXY(X[3], Y[3]);
            do{vTaskDelay(100);}while(tp.update() == 0);
            tp.gotoXY(X[4], Y[4]);
            do{vTaskDelay(100);}while(tp.update() == 0);*/


        	// DrawLines
        	tp.pushXY(X,Y,5);

            do {
            	vTaskDelay(100);
            }while(tp.update() == 0);


            while(1)
            {
                led1->Toggle();
                vTaskDelay(500);
            }
        }


        // Run
        /*pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(1.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        led1->Toggle();
        pfc->SetAngularPosition(3.141592654);
        pfc->SetLinearPosition(1.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        led1->Toggle();
        pfc->SetAngularPosition(3.141592654);
        pfc->SetLinearPosition(0.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        led1->Toggle();
        pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(0.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        led1->Toggle();
        pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(-2.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
*/
/*
        pfc->SetAngularPosition(3.141592654/2.0);
        pfc->SetLinearPosition(1.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
*/
/*
        pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(1.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        pfc->SetAngularPosition(3.141592654/2.0);
        pfc->SetLinearPosition(1.0 + 0.1*3.141592654);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(1.0 + 0.2*3.141592654);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
        pfc->SetAngularPosition(0.0);
        pfc->SetLinearPosition(1.0 + 0.2*3.141592654 + 1.0);
        vTaskDelay(500);
        while(pfc->isPositioningFinished() == false) vTaskDelay(10);
*/
        while(1)
        {
            led1->Toggle();
            vTaskDelay(500);
        }
    }
}

void TASKHANDLER_Test2 (void * obj)
{
	while(1)
		vTaskDelay(pdMS_TO_TICKS(3777u));
}

void TASKHANDLER_CLI (void * obj)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    int ch = '*';

    bool traces = false;
    robot_t r;

    float32_t linPos = 0.0, angPos = 0.0;

    // Get instances
    VelocityControl* vc = VelocityControl::GetInstance();
    PositionControl* pc = PositionControl::GetInstance();
    Odometry *odo = Odometry::GetInstance();

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        ch = getchar();
        putchar(ch);

        switch (ch)
        {
            /* Emergency Stop ! */
            case 'a':
                vc->Stop();
                printf(":STOP\r\n");
                vTaskSuspend(xHandleTraces);
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                break;

            case 't':
                if(traces == false) {
                    printf(":Enable Traces\r\n");
                    vTaskResume(xHandleTraces);
                    traces = true;
                }
                else {
                    printf(":Disable Traces\r\n");
                    vTaskSuspend(xHandleTraces);
                    traces = false;
                }
                break;

            case 'o':
                odo->GetRobot(&r);
                printf(":%d\t%d\t%.3f\r\n", r.Xmm, r.Ymm, r.Odeg);
                break;

            /*case 'x':
                printf(":linPos=");
                scanf("%f", &linPos);
                printf("%.3f\tangPos=", linPos);
                scanf("%f", &angPos);
                printf("%.3f\r\n", angPos);
                vc->Start();
                pc->SetAngularPosition(angPos);
                pc->SetLinearPosition(linPos);
                break;*/

            default:
                printf("\r\n");
                break;
        }
    }
}

char* traces[1024];

void TASKHANDLER_Traces (void * obj)
{
    TickType_t xLastWakeTime, tick;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    robot_t r;

    // Get instances
    VelocityControl* vc = VelocityControl::GetInstance();
    PositionControl* pc = PositionControl::GetInstance();
    //ProfileControl* pfc = ProfileControl::GetInstance();
    ProfileGenerator *pg = ProfileGenerator::GetInstance();
    Odometry *odometry = Odometry::GetInstance();
    //TrajectoryPlanning *trajectoy = TrajectoryPlanning::GetInstance();

    GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);

    vTaskDelay(pdMS_TO_TICKS(1003u));

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        //printf("%.3f\t%.3f\r\n",vc->GetLeftSpeed(), vc->GetLeftSpeed());

        //printf("%.3f\t%.3f\t%.3f\r\n", vc->GetAngularVelocity(), vc->GetAngularSpeed(), odometry->GetAngularVelocity());
        //printf("%.3f\t%.3f\t%.3f\r\n", vc->GetLinearVelocity(), vc->GetLinearSpeed(), odometry->GetLinearVelocity());

        //printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetAngularPosition(), vc->GetAngularVelocity(), vc->GetAngularSpeed(), odometry->GetAngularVelocity(), odometry->GetAngularPosition());
        //printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetLinearPosition(), vc->GetLinearVelocity(), vc->GetLinearSpeed(), odometry->GetLinearVelocity(), odometry->GetLinearPosition());

        odometry->GetRobot(&r);
        printf("%ld\t%ld\t%.3f\t\t", r.Xmm, r.Ymm, r.Odeg);
        /*printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t\t", pc->GetAngularPosition(), vc->GetAngularVelocity(), vc->GetAngularSpeed(), odometry->GetAngularVelocity(), odometry->GetAngularPosition());
        printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetLinearPosition(), vc->GetLinearVelocity(), vc->GetLinearSpeed(), odometry->GetLinearVelocity(), odometry->GetLinearPosition());
         */

        //printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetLinearPosition(), vc->GetLinearVelocity(), vc->GetLinearSpeed(), odometry->GetLinearVelocity(), odometry->GetLinearPosition());

        //printf("%ld\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", pfc->GetLinearPhase(), pc->GetLinearPosition(), vc->GetLinearVelocity(), vc->GetLinearSpeed(), odometry->GetLinearVelocity(), odometry->GetLinearPosition());

        //led1->Set(GPIO::State::Low);
        //printf("%ld\t%ld\t%", pTp->GetStep(), pfc->GetLinearPhase());
   //     printf("%ld\t%ld\t%ld\t", pTp->GetStep(), pg->GetLinearPhase(), pg->GetAngularPhase());
        printf("%.3f\t%.3f\t", pc->GetLinearPosition(), vc->GetLinearVelocity());
        printf("%.3f\t%.3f\t", pc->GetAngularPosition(), vc->GetAngularVelocity());
        //printf("%.3f\t%.3f\t", odometry->GetLinearVelocity(), odometry->GetLinearPosition());
   //     printf("%.3f\t%.3f\t", odometry->GetAngularVelocity(), odometry->GetAngularPosition());

		/*printf("%f", pc->GetAngularPosition());
		printf("\t");
		printf("%f", vc->GetAngularVelocity());*/

		printf("\r\n");

        //led1->Set(GPIO::State::High);

        //odometry->GetRobot(&r);
        //printf("%ld\t%ld\t%.3f\r\n", r.Xmm, r.Ymm, r.Odeg);

	    tick = xTaskGetTickCount();
    }
}



/**
 * @brief Main
 */
int main(void)
{
    HardwareInit();

    // Start (Led init and set up led1)
    GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);
    led1->Set(GPIO::State::Low);

    // Serial init
    Serial *serial0 = Serial::GetInstance(Serial::SERIAL0);



    // Welcome
    printf("\r\n\r\nS/0 CarteProp Firmware V0.1 (" __DATE__ " - " __TIME__ ")\r\n");

    // Odometry, VC, PC in standalone mode
    Odometry *odometry = Odometry::GetInstance();
    VelocityControl *vc  = VelocityControl::GetInstance();
    PositionControl *pc  = PositionControl::GetInstance();
    //ProfileControl  *pfc = ProfileControl::GetInstance();
    ProfileGenerator *pg = ProfileGenerator::GetInstance();

    // Create Test task
    xTaskCreate(&TASKHANDLER_Test,
                "Test Task",
                1024,
                NULL,
                3,
                NULL);

    // Create cli task
    xTaskCreate(&TASKHANDLER_CLI,
                "CLI Task",
                512,
                NULL,
                1,
                NULL);

    // Create Traces Task and suspend it
    xTaskCreate(&TASKHANDLER_Traces,
                "Traces Task",
                512,
                NULL,
                2,
                &xHandleTraces);
    vTaskSuspend(xHandleTraces);


    vTaskStartScheduler();

    assert(0 == 1);

    return 0;
}

/**
 * @brief Assertion failed callback
 * @param file : File name where assertion occured
 * @param line : Line where assertion occured
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    printf(ASSERT_FAILED_MESSSAGE, file, line);

    while(1)
    {

    }
}

/**
 * @brief FreeRTOS Tick Hook
 */
void vApplicationTickHook(void)
{
    // Do something
}

/**
 * @brief FreeRTOS Idle Hook
 */
void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

/**
 * @brief FreeRTOS Malloc Failed Hook
 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */

    taskDISABLE_INTERRUPTS();
    printf("ERROR | Safe malloc failed !\n");
    for( ;; );
}

/**
 * @brief FreeRTOS Stack Overflow Hook
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    printf("ERROR | %s Task Stack Overflowed !\n", pcTaskName);
    for( ;; );
}
