/**
 * @file	GPIO.hpp
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	GPIO Abstraction Class
 *
 *	HOWTO :
 *	- Get GPIO instance with GPIO::GetInstance()
 *	- Get() and Set() can be use to retrieve or set GPIO state
 *	- InterruptCallback can be used to set a function called when interrupt is raised
 */

#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include "stm32f4xx.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief GPIO Definition structure
 * Used to define GPIO port, pin, mode and, in case of input,
 * specific necessary definitions to initialize external interrupt
 */
typedef struct
{
	GPIO_TypeDef * 		PORT;		/**< GPIO Port */
	uint16_t			PIN;		/**< GPIO Pin number */
	GPIOMode_TypeDef	MODE;		/**< GPIO Mode */

	// Interrupt definitions - INPUTS ONLY
	uint8_t 			INT_PORTSOURCE;		/**< Interrupt Port */
	uint8_t				INT_PINSOURCE;		/**< Interrupt Pin */
	uint32_t			INT_LINE;			/**< Interrupt Line */
	uint32_t			INT_TRIGGER;		/**< Interrupt trigger */
	uint8_t				INT_PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
	uint8_t				INT_CHANNEL;		/**< Interrupt IRQ Channel */
}GPIO_DEF;

/*----------------------------------------------------------------------------*/
/* Class										                              */
/*----------------------------------------------------------------------------*/
/**
 * @addtogroup HAL
 */
namespace HAL
{
	/**
	 * @brief GPIO Abstraction class
	 *
	 * HOWTO :
	 *	- Get GPIO instance with GPIO::GetInstance()
	 *	- Get() and Set() can be use to retrieve or set GPIO state
	 *	- InterruptCallback can be used to set a function called when interrupt is raised
	 */
	class GPIO
	{
	public:

		/**
		 * @brief GPIO Identifier list
		 */
		enum ID
		{
			GPIO0,		//!< DIG_IN2
			GPIO1,		//!< PWM_OUT1
			GPIO2,		//!< DIG_OUT7
			GPIO3,		//!< DIG_IN3
			GPIO4,		//!< PWM_OUT3
			GPIO5,		//!< DIG_OUT8
			GPIO6,		//!< LED1
			GPIO7,		//!< LED2
			GPIO8,		//!< LED3
			GPIO9,		//!< LED4
			GPIO_MAX
		};

		/**
		 * @brief GPIO State
		 */
		enum State
		{
			Low = 0,//!< Low
			High    //!< High
		};

		/**
		 * @brief Get instance method
		 * @param id : GPIO ID
		 * @return GPIO instance
		 */
		static GPIO& GetInstance (enum ID id);

		/**
		 * @brief Return GPIO ID
		 * @return ID
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @brief Return GPIO interrupt state
		 * @return true if interrupt is enabled, false else
		 */
		bool GetInterruptState ()
		{
			return this->intState;
		}

		/**
		 * @brief Enable GPIO interrupt (if GPIO is an input)
		 * @param state : true if interrupt must be enabled, false else
		 */
		void EnableInterrupt (bool state)
		{
			this->intState = state;
		}

		/**
		 * @brief Return GPIO state
		 * @return Low if GPIO is '0' logic, High else
		 */
		enum State Get();

		/**
		 * @brief Set GPIO state (if GPIO is an output)
		 * @param state : Low if GPIO must be '0' logic, High else
		 */
		void Set(enum State state);

		/**
		 * @brief Toggle GPIO (if GPIO is an output)
		 */
		void Toggle();

		/**
		 * @brief Interrupt callback (only called if GPIO is an input).
		 * Default initialized at NULL
		 * @param obj : GPIO instance that may have raised the interrupt
		 */
		void (*InterruptCallback) (const GPIO obj);

	private:
		/**
		 * @private
		 * @brief GPIO private constructor
		 * @param id : GPIO ID
		 */
		GPIO (enum ID id);

		/**
		 * @private
		 * @brief GPIO instance
		 */
		GPIO* instance;

		/**
		 * @private
		 * @brief GPIO ID
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Interrupt state
		 */
		bool intState;

		/**
		 * @private
		 * @brief GPIO definition
		 */
		GPIO_DEF gpio;
	};
}
#endif /* INC_GPIO_HPP_ */
