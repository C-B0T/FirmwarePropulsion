/**
 * @file	Message.hpp
 * @author	Kevin Wysocki
 * @date	20 avr. 2017
 * @brief	Message Class
 *
 *	Used to encode and decode messages
 *	written or read on the I2C bus
 */

#ifndef INC_MESSAGE_HPP_
#define INC_MESSAGE_HPP_

#include "common.h"
#include "I2CCommon.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MSG_ERROR_UNKNOWN_TYPE		(-1)
#define MSG_ERROR_WRONG_NB_DATA		(-2)
#define MSG_ERROR_NO_ANSWER_NEEDED	(-3)

/**
 * @brief Message type list
 */
typedef enum
{
	MSG_TYPE_UNKNOWN				=	-1,
	MSG_TYPE_RESET 					= 	0x00,
	MSG_TYPE_BOOT_MODE				=	0x01,
	MSG_TYPE_PING					=	0x02,
	MSG_TYPE_CHANGE_ADDR			=	0x03,
	MSG_TYPE_CHECKUP				=	0x10,
	MSG_TYPE_GET_DISTANCE			=	0x11,
	MSG_TYPE_GET_POSITION			=	0x20,
	MSG_TYPE_GOTO					=	0x21,
	MSG_TYPE_SET_ANGLE				=	0x22,
	MSG_TYPE_DISABLE_POS_CONTROL	=	0x23,
	MSG_TYPE_ENABLE_POS_CONTROL		=	0x24,
}MESSAGE_TYPE;

/**
 * @brief Reset command param
 */
typedef struct
{
	uint32_t key;
}MESSAGE_PARAM_RESET;

/**
 * @brief Boot mode command param
 */
typedef struct
{
	uint32_t key;
}MESSAGE_PARAM_BOOT_MODE;

/**
 * @brief Ping command param
 */
typedef struct
{
	uint32_t key;
}MESSAGE_PARAM_PING;

/**
 * @brief Change address command param
 */
typedef struct
{
	uint8_t addr;
}MESSAGE_PARAM_CHANGE_ADDR;

/**
 * @brief Board checkup command param
 */
typedef struct
{
	uint8_t cmdStatus;
	uint8_t actStatus;
}MESSAGE_PARAM_CHECKUP;

/**
 * @brief Get distance command param
 */
typedef struct
{
	uint16_t distance;
	uint8_t status;
}MESSAGE_PARAM_GET_DISTANCE;

/**
 * @brief Get position command param
 */
typedef struct
{
	uint16_t posX;
	uint16_t posY;
	uint16_t angle;
}MESSAGE_PARAM_GET_POSITION;

/**
 * @brief Goto command param
 */
typedef struct
{
	uint16_t posX;
	uint16_t posY;
	uint8_t speed;
	uint8_t acc;
}MESSAGE_PARAM_GOTO;

/**
 * @brief Set angle command param
 */
typedef struct
{
	uint16_t angle;
	uint8_t speed;
	uint8_t acc;
}MESSAGE_PARAM_SET_ANGLE;

/**
 * @brief Commands param union
 */
typedef union
{
	MESSAGE_PARAM_RESET			Reset;
	MESSAGE_PARAM_BOOT_MODE		BootMode;
	MESSAGE_PARAM_PING			Ping;
	MESSAGE_PARAM_CHANGE_ADDR 	ChangeAddress;
	MESSAGE_PARAM_CHECKUP		Checkup;
	MESSAGE_PARAM_GET_DISTANCE	GetDistance;
	MESSAGE_PARAM_GET_POSITION	GetPosition;
	MESSAGE_PARAM_GOTO			Goto;
	MESSAGE_PARAM_SET_ANGLE		SetAngle;
}MESSAGE_PARAM;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{

	class Message
	{
	public :
		/**
		 * @brief Create a new message without message type (unknown by default)
		 */
		Message ();

		/**
		 * @brief Create a new message with a specific message type
		 * @param type : Message type
		 */
		Message (MESSAGE_TYPE type);

		/**
		 * @brief Encode message
		 * @param frame : Encoded I2C frame
		 * @return 0 if no error, < 0 else
		 */
		int32_t	Encode (I2C_FRAME * frame);

		/**
		 * @brief Decode message
		 * @param frame : Received I2C frame
		 * @return 0 if no error, < 0 else
		 */
		int32_t Decode (I2C_FRAME * frame);

		/**
		 * @brief Current message type
		 */
		MESSAGE_TYPE Type;

		/**
		 * @brief Current message param
		 */
		MESSAGE_PARAM Param;

	private :

		int32_t GetType (I2C_FRAME * frame);

		int32_t GetParam (I2C_FRAME * frame);
	};
}

#endif /* INC_MESSAGE_HPP_ */
