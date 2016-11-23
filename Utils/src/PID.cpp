/**
 * @file	PID.cpp
 * @author	Kevin WYSOCKI
 * @date	18 nov. 2016
 * @brief	PID Controller class
 *
 * HOWTO :
 * - Create a new PID with PID()
 * - Call Reset() to reset errors
 * - Call SetSetpoint() to set controller setpoint
 * - Call Get() to get the new output from a feedback value *
 */

#include "PID.hpp"

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Utils
{
	PID::PID()
	{
		this->kp		=	0.0f;
		this->ki		=	0.0f;
		this->kd		=	0.0f;
		this->dt		=	1.0f;
		this->err		=	0.0f;
		this->intErr	=	0.0f;
		this->diffErr	=	0.0f;
		this->setpoint	=	0.0f;
		this->output	=	0.0f;
	}

	PID::PID(float32_t kp, float32_t ki, float32_t kd, float32_t dt) : PID()
	{
		assert(dt > 0.0f);

		this->kp	=	kp;
		this->ki	=	ki;
		this->kd	=	kd;
		this->dt	=	dt;

	}

	void PID::Reset ()
	{
		this->err		=	0.0f;
		this->intErr	=	0.0f;
		this->diffErr	=	0.0f;
	}

	float32_t PID::Get (float32_t feedback)
	{
		float32_t err = 0.0;

		err = (this->setpoint - feedback) / this->setpoint;

		this->intErr	=	err + this->intErr;
		this->diffErr	=	err - this->err;
		this->err 		= 	err;

		// Output    =  kp * current error    + Ki * error sum * integration time    + kd * differential error / derivation time
		this->output = (this->kp * this->err) + (this->ki * this->intErr * this->dt) + (this->kd * this->diffErr / this->dt);

		return this->output;
	}
}