/**
 * @file	Event.hpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Event class
 */

#ifndef INC_EVENT_HPP_
#define INC_EVENT_HPP_

#include "Observable.hpp"

#include <vector>
#include <algorithm>

/*----------------------------------------------------------------------------*/
/* Types		   		                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Redefine observer callback
 */
typedef ObserverCallback EventCallback;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
	/**
	 * @class Event
	 * @brief Event is a concrete implementation of Observable class
	 *
	 * HOWTO:
	 * 	An object can subscribe to an event raised by another object using "+=" operator
	 * 	and unsubscribe to it using '-=' operator.
	 */
	class Event : protected Utils::Observable
	{
	public:

		/**
		 * @brief Event constructor
		 */
		Event();

		/**
		 * @brief Add a new event callback to the callback list
		 * @param cb : Event callback
		 * @return Event object reference
		 */
		Event& operator += (EventCallback cb);

		/**
		 * @brief Remove a callback from the callback list
		 * @param cb : Event callback
		 * @return Event object reference
		 */
		Event& operator -= (EventCallback cb);
	};
}

#endif /* INC_EVENT_HPP_ */