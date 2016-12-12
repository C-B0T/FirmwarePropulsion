/**
 * @file	Odometry.cpp
 * @author	Romain DESILLE
 * @date	7 déc. 2016
 * @brief	Odometry class
 */

#include "Odometry.hpp"
#include "common.h"

using namespace HAL;
using namespace Utils;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ODOMETRY_WD          47.7                // Wheel diameter
#define ODOMETRY_WC          1.00000000000       // Wheel correction (Difference between the (referent) right and left wheel)
#define ODOMETRY_ER          4096                // Encoder resolution
#define ODOMETRY_ADW         254                 // Axial distance between wheels

#define _PI_        3.14159265359       // PI
#define _2_PI_      6.28318530718       // 2*PI

#define ODOMETRY_TICK_BY_MM  27.3332765998       // (ER/(_PI_*WD))
#define ODOMETRY_AWD_TICK    6942,65225634       // (ADW * TICK_BY_MM)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Odometry* _odometry = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Location
{
	Odometry* Odometry::GetInstance(void)
	{
		if(_odometry != NULL)
		{
			return _odometry;
		}
		else
		{
			_odometry = new Odometry();
			return _odometry;
		}
	}

	Odometry::Odometry(void)
	{
		this->name = "ODOMETRY";
		this->Data.LeftVelocity 	= 0;
		this->Data.RigthVelocity 	= 0;
		this->Data.AngularVelocity  = 0;
		this->Data.LinearVelocity   = 0;
		this->Data.X 				= 0;
		this->Data.Y 				= 0;
		this->Data.O 				= 0;
		this->Data.L 				= 0;
		this->Data.Xmm 				= 0;
		this->Data.Ymm 				= 0;
		this->Data.Odeg 			= 0;

		// Init encoder
		this->leftEncoder  = Encoder::GetInstance(Encoder::ID::ENCODER0);
		this->rightEncoder = Encoder::GetInstance(Encoder::ID::ENCODER1);
	}


	void Odometry::Compute(void)
	{
			this->Data.LeftVelocity  = this->leftEncoder->GetRelativeValue();
			this->Data.RigthVelocity = this->rightEncoder->GetRelativeValue();

			this->Data.AngularVelocity  = this->Data.RigthVelocity - this->Data.LeftVelocity;
			this->Data.LinearVelocity   = (this->Data.RigthVelocity + this->Data.LeftVelocity)/2.0;

			this->Data.O += this->Data.AngularVelocity / ODOMETRY_AWD_TICK;

			while(this->Data.O > _2_PI_)
				this->Data.O -= _2_PI_;
			while(this->Data.O < -_2_PI_)
				this->Data.O += _2_PI_;

			this->Data.L += this->Data.LinearVelocity;

			this->Data.X += cosf(this->Data.O) * this->Data.LinearVelocity;
			this->Data.Y += sinf(this->Data.O) * this->Data.LinearVelocity;


			this->Data.Xmm = this->Data.X / ODOMETRY_TICK_BY_MM;
			this->Data.Ymm = this->Data.Y / ODOMETRY_TICK_BY_MM;
			this->Data.Odeg = (180.0 * this->Data.O)/_PI_;
	}


}
