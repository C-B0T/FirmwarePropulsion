/**
 * @file	Message.hpp
 * @author	Kevin Wysocki
 * @date	20 avr. 2017
 * @brief	Message Class
 *
 *	Used to encode and decode messages
 *	written or read on the I2C bus
 */

#include <stddef.h>
#include "Message.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MSG_FRAME_INDEX_OPCODE		(0u)
#define MSG_FRAME_INDEX_NB_DATA		(1u)
#define MSG_FRAME_INDEX_FIRST_DATA	(2u)

// Message nb of data bytes - Encode
#define MSG_NB_DATA_ENCODE_RESET					(0u)
#define MSG_NB_DATA_ENCODE_BOOT_MODE				(0u)
#define MSG_NB_DATA_ENCODE_PING						(4u)
#define MSG_NB_DATA_ENCODE_CHANGE_ADDR				(0u)
#define MSG_NB_DATA_ENCODE_CHECKUP					(2u)
#define MSG_NB_DATA_ENCODE_GET_POSITION				(6u)
#define MSG_NB_DATA_ENCODE_GOTO						(0u)
#define MSG_NB_DATA_ENCODE_GET_DISTANCE				(3u)
#define MSG_NB_DATA_ENCODE_SET_ANGLE				(0u)
#define MSG_NB_DATA_ENCODE_DISABLE_POS_CONTROL		(0u)
#define MSG_NB_DATA_ENCODE_ENABLE_POS_CONTROL		(0u)

// Message nb of data bytes - Decode
#define MSG_NB_DATA_DECODE_RESET					(4u)
#define MSG_NB_DATA_DECODE_BOOT_MODE				(4u)
#define MSG_NB_DATA_DECODE_PING						(0u)
#define MSG_NB_DATA_DECODE_CHANGE_ADDR				(1u)
#define MSG_NB_DATA_DECODE_CHECKUP					(0u)
#define MSG_NB_DATA_DECODE_GET_POSITION				(0u)
#define MSG_NB_DATA_DECODE_GOTO						(6u)
#define MSG_NB_DATA_DECODE_GET_DISTANCE				(0u)
#define MSG_NB_DATA_DECODE_SET_ANGLE				(4u)
#define MSG_NB_DATA_DECODE_DISABLE_POS_CONTROL		(0u)
#define MSG_NB_DATA_DECODE_ENABLE_POS_CONTROL		(0u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _decode_Reset (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	param->Reset.key	 =	(uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
	param->Reset.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
	param->Reset.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 16u);
	param->Reset.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 24u);
}

static void _decode_BootMode (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	param->BootMode.key	 =	(uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 16u);
	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 24u);
}

static void _decode_ChangeAddr (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	param->ChangeAddress.addr	 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_Goto (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	param->Goto.posX	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
	param->Goto.posX	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
	param->Goto.posY	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u];
	param->Goto.posY	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 8u);
	param->Goto.speed	 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 4u];
	param->Goto.acc		 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 5u];
}

static void _decode_SetAngle (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	param->SetAngle.angle	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
	param->SetAngle.angle	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
	param->SetAngle.speed	 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u];
	param->SetAngle.acc		 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u];
}

static void _encode_Ping (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	assert(frame != NULL);
	assert(param != NULL);

	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 0u]	=	(uint8_t)(param->Ping.key & 0x000000FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u]	=	(uint8_t)((param->Ping.key >> 8u) & 0x000000FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u]	=	(uint8_t)((param->Ping.key >> 16u) & 0x000000FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u]	=	(uint8_t)((param->Ping.key >> 24u) & 0x000000FFu);
	frame->Length++;
}

static void _encode_Checkup(I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 0u]	=	param->Checkup.cmdStatus;
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u]	=	param->Checkup.cmdStatus;
	frame->Length++;
}

static void _encode_GetDistance (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 0u]	=	(uint8_t)(param->GetDistance.distance & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u]	=	(uint8_t)((param->GetDistance.distance >> 8u) & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u]	=	param->GetDistance.status;
	frame->Length++;
}

static void _encode_GetPosition (I2C_FRAME * frame, MESSAGE_PARAM * param)
{
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 0u]	=	(uint8_t)(param->GetPosition.posX & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u]	=	(uint8_t)((param->GetPosition.posX >> 8u) & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u]	=	(uint8_t)(param->GetPosition.posY & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u]	=	(uint8_t)((param->GetPosition.posY >> 8u) & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 4u]	=	(uint8_t)(param->GetPosition.angle & 0x00FFu);
	frame->Length++;
	frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 5u]	=	(uint8_t)((param->GetPosition.angle >> 8u) & 0x00FFu);
	frame->Length++;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	Message::Message()
	{
		this->Type = MSG_TYPE_UNKNOWN;
	}

	Message::Message(MESSAGE_TYPE type)
	{
		this->Type = type;
	}

	int32_t Message::Decode(I2C_FRAME * frame)
	{
		int32_t rval = NO_ERROR;

		assert(frame != NULL);

		// 1. Get message type
		if(rval == NO_ERROR)
		{
			rval = this->GetType(frame);
		}

		// 2. Get param
		if(rval == NO_ERROR)
		{
			rval = this->GetParam(frame);
		}

		return rval;
	}

	int32_t Message::Encode(I2C_FRAME * frame)
	{
		int32_t rval = 0;

		assert(frame != NULL);

		// 1. Set Opcode field
		if(rval == NO_ERROR)
		{
			if(this->Type != MSG_TYPE_UNKNOWN)
			{
				frame->Data[MSG_FRAME_INDEX_OPCODE]	=	(uint8_t)this->Type;
				frame->Length++;
			}
			else
			{
				rval = MSG_ERROR_UNKNOWN_TYPE;
			}
		}

		// 2. Set Nb Data field
		if(rval == NO_ERROR)
		{
			switch(this->Type)
			{
			case MSG_TYPE_PING:
				frame->Data[MSG_FRAME_INDEX_NB_DATA]=	MSG_NB_DATA_ENCODE_PING;
				frame->Length++;
				break;
			case MSG_TYPE_CHECKUP:
				frame->Data[MSG_FRAME_INDEX_NB_DATA]=	MSG_NB_DATA_ENCODE_CHECKUP;
				frame->Length++;
				break;
			case MSG_TYPE_GET_DISTANCE:
				frame->Data[MSG_FRAME_INDEX_NB_DATA]=	MSG_NB_DATA_ENCODE_GET_DISTANCE;
				frame->Length++;
				break;
			case MSG_TYPE_GET_POSITION:
				frame->Data[MSG_FRAME_INDEX_NB_DATA]=	MSG_NB_DATA_ENCODE_GET_POSITION;
				frame->Length++;
				break;

			// These command doesn't need an answer
			case MSG_TYPE_RESET:
			case MSG_TYPE_BOOT_MODE:
			case MSG_TYPE_CHANGE_ADDR:
			case MSG_TYPE_GOTO:
			case MSG_TYPE_SET_ANGLE:
			case MSG_TYPE_DISABLE_POS_CONTROL:
			case MSG_TYPE_ENABLE_POS_CONTROL:
				rval = MSG_ERROR_NO_ANSWER_NEEDED;
				break;
			}
		}

		// 3. Encode data
		if(rval == NO_ERROR)
		{
			switch(this->Type)
			{
			case MSG_TYPE_PING:
				_encode_Ping(frame, &this->Param);
				break;
			case MSG_TYPE_CHECKUP:
				_encode_Checkup(frame, &this->Param);
				break;
			case MSG_TYPE_GET_DISTANCE:
				_encode_GetDistance(frame, &this->Param);
				break;
			case MSG_TYPE_GET_POSITION:
				_encode_GetPosition(frame, &this->Param);
				break;

			// These command doesn't need an answer
			case MSG_TYPE_RESET:
			case MSG_TYPE_BOOT_MODE:
			case MSG_TYPE_CHANGE_ADDR:
			case MSG_TYPE_GOTO:
			case MSG_TYPE_SET_ANGLE:
			case MSG_TYPE_DISABLE_POS_CONTROL:
			case MSG_TYPE_ENABLE_POS_CONTROL:
				rval = MSG_ERROR_NO_ANSWER_NEEDED;
				break;
			}
		}

		return rval;
	}

	int32_t Message::GetType(I2C_FRAME * frame)
	{
		int32_t rval = 0;

		assert(frame != NULL);

		switch(frame->Data[MSG_FRAME_INDEX_OPCODE])
		{
		case MSG_TYPE_RESET:
		case MSG_TYPE_BOOT_MODE:
		case MSG_TYPE_PING:
		case MSG_TYPE_CHANGE_ADDR:
		case MSG_TYPE_CHECKUP:
		case MSG_TYPE_GET_DISTANCE:
		case MSG_TYPE_GET_POSITION:
		case MSG_TYPE_GOTO:
		case MSG_TYPE_SET_ANGLE:
		case MSG_TYPE_DISABLE_POS_CONTROL:
		case MSG_TYPE_ENABLE_POS_CONTROL:
			this->Type = (MESSAGE_TYPE)frame->Data[MSG_FRAME_INDEX_OPCODE];
			break;
		default:
			this->Type = MSG_TYPE_UNKNOWN;
			rval = MSG_ERROR_UNKNOWN_TYPE;
			break;
		}

		return rval;
	}

	int32_t Message::GetParam(I2C_FRAME * frame)
	{
		int32_t rval = NO_ERROR;

		// 1. Check Nb data
		if(rval == NO_ERROR)
		{
			switch(this->Type)
			{
			case MSG_TYPE_RESET:
				if(frame->Length != MSG_NB_DATA_DECODE_RESET)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_BOOT_MODE:
				if(frame->Length != MSG_NB_DATA_DECODE_BOOT_MODE)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_PING:
				if(frame->Length != MSG_NB_DATA_DECODE_PING)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_CHANGE_ADDR:
				if(frame->Length != MSG_NB_DATA_DECODE_CHANGE_ADDR)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_CHECKUP:
				if(frame->Length != MSG_NB_DATA_DECODE_CHECKUP)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_GET_DISTANCE:
				if(frame->Length != MSG_NB_DATA_DECODE_GET_DISTANCE)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_GET_POSITION:
				if(frame->Length != MSG_NB_DATA_DECODE_GET_POSITION)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_GOTO:
				if(frame->Length != MSG_NB_DATA_DECODE_GOTO)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_SET_ANGLE:
				if(frame->Length != MSG_NB_DATA_DECODE_SET_ANGLE)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_DISABLE_POS_CONTROL:
				if(frame->Length != MSG_NB_DATA_DECODE_DISABLE_POS_CONTROL)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
			case MSG_TYPE_ENABLE_POS_CONTROL:
				if(frame->Length != MSG_NB_DATA_DECODE_ENABLE_POS_CONTROL)
					rval = MSG_ERROR_WRONG_NB_DATA;
				break;
				break;
			default:
				rval = MSG_ERROR_UNKNOWN_TYPE;
				break;
			}
		}

		// 2. Decode param
		if((rval == NO_ERROR) && (frame->Length > 0u))
		{
			switch(this->Type)
			{
			case MSG_TYPE_RESET:
				_decode_Reset(frame, &this->Param);
				break;
			case MSG_TYPE_BOOT_MODE:
				_decode_BootMode(frame, &this->Param);
				break;
			case MSG_TYPE_CHANGE_ADDR:
				_decode_ChangeAddr(frame, &this->Param);
				break;
			case MSG_TYPE_GOTO:
				_decode_Goto(frame, &this->Param);
				break;
			case MSG_TYPE_SET_ANGLE:
				_decode_SetAngle(frame, &this->Param);
				break;
			default:	// Other messages doesn't need param
				break;
			}
		}

		return rval;
	}
}
