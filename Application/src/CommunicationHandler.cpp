/**
 * @file	Communication.cpp
 * @author	kevin.wysocki
 * @date	21 avr. 2017
 * @brief	
 *
 *
 */

#include "Communication.hpp"
#include <string.h>

// FreeRTOS API
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define COM_EVENT_DATA_RECEIVED		(0x01u)
#define COM_EVENT_DATA_REQUESTED	(0x02u)
#define COM_EVENT_MASK				(COM_EVENT_DATA_RECEIVED | \
									 COM_EVENT_DATA_REQUESTED)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Communication::Communication * _instance = NULL;

static EventGroupHandle_t _eventHandle;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _onDataReceived (void * obj)
{

}

static void _onDataRequested (void * obj)
{

}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	Communication * Communication::GetInstance()
	{
		if(_instance == NULL)
		{
			_instance = new Communication();

			return _instance;
		}
		else
		{
			return _instance;
		}
	}

	Communication::Communication()
	{
		// Create I2C bus
		this->bus = HAL::I2CSlave::GetInstance(HAL::I2CSlave::I2C_SLAVE0);

		this->bus->DataReceived += _onDataReceived;
		this->bus->DataRequest += _onDataRequested;

		// Create Communication task
		xTaskCreate((TaskFunction_t)&Communication::TaskHandler,
					"Communication Task",
					128,
					NULL,
					configMAX_PRIORITIES - 1u,		// Communication must be higher priority task to avoid bus overrun
					NULL);

		// Create event
		_eventHandle = xEventGroupCreate();
	}

	int32_t Communication::Write (Message * msg)
	{
		int32_t rval = NO_ERROR;

		assert(msg != NULL);

		if(rval == NO_ERROR)
		{
			rval = msg->Encode(&this->frame);
		}

		if(rval == NO_ERROR)
		{
			rval = this->bus->Write(&this->frame);
		}

		return rval;
	}

	void Communication::GetLastMessage (Message * msg)
	{
		memcpy(msg, &this->msg, sizeof(Message));
	}

	void Communication::TaskHandler(void * obj)
	{
		EventBits_t events = 0u;
		int32_t error = NO_ERROR;

		while(1)
		{
			// 1. Wait event
			events = xEventGroupWaitBits(_eventHandle,
										 COM_EVENT_MASK,
										 pdTRUE,
										 pdFALSE,
										 portMAX_DELAY);

			if( ((events & COM_EVENT_DATA_RECEIVED) == COM_EVENT_DATA_RECEIVED) ||
				((events & COM_EVENT_DATA_REQUESTED) == COM_EVENT_DATA_REQUESTED) )
			{
				// Get I2C frame
				if(error == NO_ERROR)
				{
					error = this->bus->Read(&this->frame);
				}

				// Decode message
				if(error == NO_ERROR)
				{
					error = this->msg.Decode(&this->frame);
				}

				// Notify
				if(error == NO_ERROR)
				{
					this->MessageReceived();
				}
			}

			error = NO_ERROR;
		}

	}
}
