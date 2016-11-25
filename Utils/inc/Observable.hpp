/**
 * @file	Observable.hpp
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

#ifndef INC_OBSERVABLE_HPP_
#define INC_OBSERVABLE_HPP_

#include <vector>
#include <algorithm>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief Observer callback
 * @param obj : any object instance
 */
typedef void (*ObserverCallback) (void * obj);

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
	/**
	 * @class Observable
	 *
	 * Abstract Observable class can be used to notify objects called "obervers".\n
	 * To be notified, an "observer" has to register itself by calling Subscribe() method
	 * passing as argument a callback which will be called when the observable notify its obersvers.
	 * Unsubscription can be achieved by calling Unsubscribe() method.
	 */
	class Observable
	{
	public:

		/**
		 * @brief Default constructor;
		 */
		Observable();

		/**
		 * @pure
		 * @brief Virtual pure destructor
		 */
		virtual ~Observable() = 0;

		/**
		 * @brief Subscribe to observable notifications
		 * @param cb : Callback to call on observable notifications
		 */
		void Subscribe (ObserverCallback cb);


		/**
		 * @brief Unsubscribe to observable notifications
		 * @param cb : Callback which where called on observable notifications
		 */
		void Unsubscribe (ObserverCallback cb);

	protected:

		/**
		 * @brief Notification method - Used by child class
		 * @param observable
		 * @param obj
		 */
		void notify (Observable& observable, void * obj);
	private:

		/**
		 * @private
		 * @brief Observer callback list used by notifications
		 */
		std::vector<ObserverCallback> observerCallbackList;
	};
}

#endif /* INC_OBSERVABLE_HPP_ */
