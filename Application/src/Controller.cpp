/**
 * @file	Controller.hpp
 * @author	Kevin WYSOCKI
 * @date	30 avr. 2017
 * @brief	Controller class
 */

#include "Controller.hpp"

#include "stm32f4xx.h"

// FreeRTOS API
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CONTROLLER_EVENT_MSG_RECEIVED		(0x01u)
#define CONTROLLER_EVENT_MASK				(CONTROLLER_EVENT_MSG_RECEIVED)

#define CONTROLLER_UNLOCK_KEY				(0x5AA555AAu)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Controller * _instance = NULL;

static EventGroupHandle_t _eventHandle;
static TaskHandle_t _taskHandle = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _onMessageReceive (void * obj)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xEventGroupSetBitsFromISR(_eventHandle, CONTROLLER_EVENT_MSG_RECEIVED, &xHigherPriorityTaskWoken);
}

static void _taskHandler (void * obj)
{
	Controller* controller = static_cast<Controller*>(obj);

	controller->TaskHandler();
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

Controller * Controller::GetInstance()
{
	if(_instance == NULL)
	{
		_instance = new Controller();

		return _instance;
	}
	else
	{
		return _instance;
	}
}

Controller::Controller ()
{
	// Create communication handler
	this->comHandler = Communication::CommunicationHandler::GetInstance();
	this->comHandler->MessageReceived += _onMessageReceive;

	// Create event
	_eventHandle = xEventGroupCreate();

	// Create Communication task
	xTaskCreate((TaskFunction_t)_taskHandler,
				"Controller Task",
				128,
				(void*)this,
				0,
				&_taskHandle);
}

void Controller::TaskHandler (void)
{
	EventBits_t events = 0u;
	int32_t error = NO_ERROR;

	while(1)
	{
		// 1. Wait event
		events = xEventGroupWaitBits(_eventHandle,
									 CONTROLLER_EVENT_MASK,
									 pdTRUE,
									 pdFALSE,
									 portMAX_DELAY);

		if((events & CONTROLLER_EVENT_MSG_RECEIVED) == CONTROLLER_EVENT_MSG_RECEIVED)
		{
			// 1. get message
			if(error == NO_ERROR)
			{
				this->comHandler->GetLastMessage(&this->msg);
			}

			// Handle message
			if(error == NO_ERROR)
			{
				switch(this->msg.Type)
				{
				case MSG_TYPE_RESET :
					error = this->Reset();
					break;
				case MSG_TYPE_BOOT_MODE :
					error = this->Bootloader();
					break;
				case MSG_TYPE_PING :
					error = this->Ping();
					break;
				case MSG_TYPE_CHANGE_ADDR :
					error = this->ChangeAddress();
					break;
				case MSG_TYPE_CHECKUP :
					error = this->Checkup();
					break;
				case MSG_TYPE_GET_POSITION :
					error = this->GetPosition();
					break;
				case MSG_TYPE_GOTO :
					error = this->Goto();
					break;
				case MSG_TYPE_SET_ANGLE :
					error = this->SetAngle();
					break;
				case MSG_TYPE_DISABLE_POS_CONTROL :
					error = this->DisablePosControl();
					break;
				case MSG_TYPE_ENABLE_POS_CONTROL :
					error = this->EnablePosControl();
					break;
				default:
					break;
				}
			}
		}

		error = NO_ERROR;
	}
}

int32_t Controller::Reset (void)
{
	int32_t error = NO_ERROR;

	if((this->msg.Type == MSG_TYPE_RESET) && (this->msg.Param.Reset.key == CONTROLLER_UNLOCK_KEY))
	{
		NVIC_SystemReset();
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::Bootloader (void)
{
	int32_t error = NO_ERROR;

	if((this->msg.Type == MSG_TYPE_BOOT_MODE) && (this->msg.Param.Reset.key == CONTROLLER_UNLOCK_KEY))
	{
		/** @todo Jump to bootloader address */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::Ping (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type != MSG_TYPE_PING)
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	// 1. Set message
	if(error == NO_ERROR)
	{
		this->msg.Type						=	MSG_TYPE_PING;
		this->msg.Param.Ping.key			=	CONTROLLER_UNLOCK_KEY;
	}

	// 2. Send message
	if(error == NO_ERROR)
	{
		error = this->comHandler->Write(&this->msg);
	}

	return error;
}

int32_t Controller::ChangeAddress (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_CHANGE_ADDR)
	{
		/**@todo Change I2C address */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::Checkup (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_CHECKUP)
	{
		/**@todo Do checkup */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::GetPosition (void)
{
	int32_t error = NO_ERROR;
	int16_t posX = 0x0E09, posY = 0x3F56, angle = 0xFE78;

	// 1. Get position
	if(error == NO_ERROR)
	{
		/**@todo Get position */
	}

	// 2. Set message
	if(error == NO_ERROR)
	{
		this->msg.Type						=	MSG_TYPE_GET_POSITION;
		this->msg.Param.GetPosition.posY 	= 	posY;
		this->msg.Param.GetPosition.posX 	= 	posX;
		this->msg.Param.GetPosition.angle 	= 	angle;
	}

	// 3. Send message
	if(error == NO_ERROR)
	{
		error = this->comHandler->Write(&this->msg);
	}

	return error;
}

int32_t Controller::Goto (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_GOTO)
	{
		/**@todo Do goto */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::SetAngle (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_SET_ANGLE)
	{
		/**@todo Do set angle */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::DisablePosControl (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_DISABLE_POS_CONTROL)
	{
		/**@todo Do disable position control */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

int32_t Controller::EnablePosControl (void)
{
	int32_t error = NO_ERROR;

	if(this->msg.Type == MSG_TYPE_ENABLE_POS_CONTROL)
	{
		/**@todo Do enable position control */
	}
	else
	{
		error = CONTROLLER_ERROR_WRONG_PARAM;
	}

	return error;
}

