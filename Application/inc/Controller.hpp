/**
 * @file	Controller.hpp
 * @author	Kevin WYSOCKI
 * @date	30 avr. 2017
 * @brief	Controller class
 */

#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include "common.h"
#include "CommunicationHandler.hpp"
#include "Message.hpp"

#include "Odometry.hpp"


/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CONTROLLER_ERROR_WRONG_PARAM	(-1)

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Controller class
 */
class Controller
{
public:
	/**
	 * @brief Return Controller instance
	 */
	static Controller * GetInstance();

	/**
	 * @private
	 * @brief Controller task handler
	 */
	void TaskHandler (void);

private:

	/**
	 * @private
	 * @brief Private constructor
	 */
	Controller ();

	/**
	 * @private
	 * @brief Handle Reset request
	 * @return Error code
	 */
	int32_t Reset (void);

	/**
	 * @private
	 * @brief Handle Bootloader mode request
	 * @return Error code
	 */
	int32_t Bootloader (void);

	/**
	 * @private
	 * @brief Handle Ping request
	 * @return Error code
	 */
	int32_t Ping (void);

	/**
	 * @private
	 * @brief Handle Change I2C Address request
	 * @return Error code
	 */
	int32_t ChangeAddress (void);

	/**
	 * @private
	 * @brief Handle Checkup request
	 * @return Error code
	 */
	int32_t Checkup (void);

	/**
	 * @private
	 * @brief Handle Get Position request
	 * @return Error code
	 */
	int32_t GetPosition (void);

	/**
	 * @private
	 * @brief Handle Goto request
	 * @return Error code
	 */
	int32_t Goto (void);

	/**
	 * @private
	 * @brief Handle Set Angle request
	 * @return Error code
	 */
	int32_t SetAngle (void);

	/**
	 * @private
	 * @brief Handle Disable Position Control request
	 * @return Error code
	 */
	int32_t DisablePosControl (void);

	/**
	 * @private
	 * @brief Handle Enable Position Control request
	 * @return Error code
	 */
	int32_t EnablePosControl (void);

	/**
	 * @private
	 * @brief Communication handler
	 */
	Communication::CommunicationHandler* comHandler;

	/**
	 * @private
	 * @brief Current message
	 */
	Communication::Message msg;

	/**@todo Add position control object */

};

#endif /* INC_CONTROLLER_HPP_ */
