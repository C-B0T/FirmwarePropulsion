/**
 * @file	Observable.cpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Observable class
 */

#include "Observable.hpp"
#include "common.h"

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

	Observable::~Observable()
	{
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
