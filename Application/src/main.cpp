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

using namespace HAL;
using namespace Utils;

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
}

/**
 * @brief Main task handler
 */
void TASKHANDLER_Test (void * obj)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	Odometry *odometry = Odometry::GetInstance();
	robot_t r;

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		odometry->GetRobot(&r);
		printf("X:%ld\tY:%ld\tO:%f\r\n",(int32_t)r.X, (int32_t)r.Y, (float32_t)r.O);
		printf("X:%ld\tY:%ld\tO:%f\r\n",(int32_t)r.X_mm, (int32_t)r.Y_mm, (float32_t)r.O_deg);
		printf("\r\n");
	}
}

/**
 * @brief Main
 */
int main(void)
{
	HardwareInit();

	// Start
	GPIO *led1 = GPIO::GetInstace(GPIO::GPIO6);
	led1->Set(GPIO::LOW);

	// Welcome
	printf("\r\n\r\nS/0 CarteProp Firmware V0.0 (" __DATE__ " - " __TIME__ ")\r\n");

	Odometry *odometry = Odometry::GetInstance();

	xTaskCreate(&TASKHANDLER_Test,
				"Test Task",
				128,
				NULL,
				1,
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
