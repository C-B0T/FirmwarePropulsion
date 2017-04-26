/**
 * @file	Communication.hpp
 * @author	Kevin WYSOCKI
 * @date	20 avr. 2017
 * @brief	
 */

#ifndef INC_COMMUNICATION_HPP_
#define INC_COMMUNICATION_HPP_

#include "common.h"
#include "I2CSlave.hpp"
#include "Message.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	class Communication
	{
	public :
		static Communication * GetInstance ();

		int32_t Write (Message * msg);

		void GetLastMessage (Message * msg);

		Utils::Event MessageReceived;

	private :

		Communication();

		void TaskHandler (void * obj);

		Message msg;

		I2C_FRAME frame;

		HAL::I2CSlave * bus;
	};
}

#endif /* INC_COMMUNICATION_HPP_ */
