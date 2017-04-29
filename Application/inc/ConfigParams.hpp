/**
 * @file    ConfigParams.hpp
 * @author  Jeremy ROULLAND
 * @date    23 mar. 2017
 * @brief   Configuration Parameters (non volatile) class
 */

#ifndef INC_CONFIGPARAMS_HPP_
#define INC_CONFIGPARAMS_HPP_

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
    /**
    * @class ConfigParams
    * @brief Backup in SRAM of the STM32F4
    *
    * HOWTO : EEPROM emulation vs Backup SRAM (on VBAT)
    * A-EEPROM:
    *   - 
    * B-SRAM:
    *   - Enable the PWR clock
    *       RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    *   - Enable access to the backup domain
    *       PWR_BackupAccessCmd(ENABLE);
    *   - Enable backup SRAM Clock
    *       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
    *   - Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode
    *       PWR_BackupRegulatorCmd(ENABLE);
    *   Nota: If you want until the Backup SRAM lower power Regulator is ready
    *       while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET) {}
    *
    */

    class ConfigParams
    {
    public:
        /**
         * @brief ConfigParams default constructor
         */
         ConfigParams();

    protected:
        /**
         * @brief Upgrade the ConfigParams
         */
         void Upgrade();

         /**
          * @brief Reset the ConfigParams by default
          */
         void Reset();

    private:
    };
}

#endif /* INC_CONFIGPARAMS_HPP_ */
