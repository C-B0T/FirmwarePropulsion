/**
 * @file	main.cpp
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	Main
 */

#include <stdio.h>

#include "../../STM32_Driver/inc/stm32f4xx.h"
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"

#include "HAL.hpp"
#include "BrushlessMotor.hpp"
#include "Odometry.hpp"
#include "VelocityControl.hpp"

using namespace HAL;
using namespace Utils;
using namespace Location;
using namespace MotionControl;


int __io_putchar(int ch)
{
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  /*while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET)
  {}*/

  return ch;
}

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

	// Enable Timer clock and USART1 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5,
						   ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_USART1,
						   ENABLE);

}

/**
 * @brief Main task handler
 */
void TASKHANDLER_Test (void * obj)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	float32_t speed = 0.0;

	// Get instances
	GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);
	VelocityControl *vc = VelocityControl::GetInstance();

	// Wait 3sec before start
	vTaskDelay(pdMS_TO_TICKS(3000u));

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

    	if(speed < 200.0)
    		speed += 0.1;

    	vc->SetAngularVelocity(speed);

    	led1->Toggle();
	}
}

/**
 * @brief Main task handler
 */
void TASKHANDLER_VelocityLoop (void * obj)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(10);

	// Get instances
	VelocityControl* vc = VelocityControl::GetInstance();

	// Wait 3sec before start
	vTaskDelay(pdMS_TO_TICKS(3000u));


	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Compute VelocityControl (10ms)
		vc->Compute(10);
	}
}

void TASKHANDLER_CLI (void * obj)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	int ch = '*';
	float32_t kp = 0.06;
	float32_t ki = 0.0;


	// Get instances
	VelocityControl* vc = VelocityControl::GetInstance();

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		ch = getchar();
		putchar(ch);

		switch (ch)
		{
			case 'p':
				kp += 0.01;
				vc->SetAngularKp(kp);
				printf(":%.3f\r\n", kp);
				break;

			case 'm':
				kp -= 0.01;
				vc->SetAngularKp(kp);
				printf(":%.3f\r\n", kp);
				break;

			default:
				printf("\r\n");
				break;
		}
	}
}


void TASKHANDLER_Traces (void * obj)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);

	// Get instances
	VelocityControl* vc = VelocityControl::GetInstance();
	Odometry *odometry = Odometry::GetInstance();

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		printf("%.3f\t%.3f\r\n",vc->GetAngularVelocity(), odometry->GetAngularVelocity(10.0));
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
	printf("\r\n\r\nS/0 CarteProp Firmware V0.0 (" __DATE__ " - " __TIME__ ")\r\n");

	// Odometry init in standalone mode
	Odometry *odometry = Odometry::GetInstance(true);

	// Create Test task
	xTaskCreate(&TASKHANDLER_Test,
				"Test Task",
				512,
				NULL,
				2,
				NULL);

	// Create Test task
	xTaskCreate(&TASKHANDLER_CLI,
				"CLI Task",
				512,
				NULL,
				1,
				NULL);

	// Create Test velocity Task
	xTaskCreate(&TASKHANDLER_VelocityLoop,
				"Velocity Task",
				512,
				NULL,
				125,
				NULL);

	// Create Test velocity Task
	xTaskCreate(&TASKHANDLER_Traces,
				"Traces Task",
				512,
				NULL,
				125,
				NULL);


	vTaskStartScheduler();

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

