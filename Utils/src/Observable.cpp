/**
 * @file	Observable.cpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Observable class
 *
 *HOWTO :
 * Abstract Observable class can be used to notify objects called "obervers".\n
 * To be notified, an "observer" has to register itself by calling Subscribe() method
 * passing as argument a callback which will be called when the observable notify its obersvers.
 * Unsubscription can be achieved by calling Unsubscribe() method.
 */

#include "Observable.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Utils
{
	Observable::Observable()
	{
		this->observerCallbackList = std::vector<ObserverCallback>();
	}

	void Observable::Subscribe(ObserverCallback cb)
	{
		this->observerCallbackList.push_back(cb);
	}

	void Observable::Unsubscribe (ObserverCallback cb)
	{
		auto it = std::find(this->observerCallbackList.begin(), this->observerCallbackList.end(), cb);

		if(it != this->observerCallbackList.end())
		{
			this->observerCallbackList.erase(it);
		}
	}

	void Observable::notify (Observable& observable, void * obj)
	{
		for(unsigned int i=0; i<observable.observerCallbackList.size(); i++)
		{
			observable.observerCallbackList[i](obj);
		}
	}
}
