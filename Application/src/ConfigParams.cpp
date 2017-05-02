/**
 * @file    ConfigParams.cpp
 * @author  Jeremy ROULLAND
 * @date    23 mar. 2017
 * @brief   Configuration Parameters (non volatile) class
 */

#include "ConfigParams.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
#define CONFIGPARAMS_MAGIC      0x80CAFE01

// Reserved=Alway0x00   Maj=0x00 Min=0x00 Revision=0x01
#define CONFIGPARAMS_VERSION    0x00000001

#define CONFIGPARAMS_INDEX_VEL_KP
#define CONFIGPARAMS_INDEX_VEL_KI
#define CONFIGPARAMS_INDEX_VEL_KD


/*----------------------------------------------------------------------------*/
/* Mapping                                                                    */
/*----------------------------------------------------------------------------*/

/*          +------------32-Bit------------+
 *  0x0000  |         Magic Number         |
 *          +------------------------------+
 *  0x0004  |     ConfigParams Version     |
 *          +------------------------------+
 *  0x0008  |           reserved           |    (0x55555555)
 *          +------------------------------+
 *  0x000C  |           reserved           |    (0xAAAAAAAA)
 *          +------------------------------+
 *  0x0010  |           <empty>            |
 *          +------------------------------+
 *  0x0014  |           <empty>            |
 *          +------------------------------+
 *  0x0018  |           <empty>            |
 *          +------------------------------+
 *  0x001C  |           <empty>            |
 *          +------------------------------+
 *  0x0020  |           <empty>            |
 *          +------------------------------+
 */


/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/


namespace Utils
{
    ConfigParams::ConfigParams()
    {
        /*
        // 1- Check if ConfigParams hasn't been initialized
        if(*(__IO uint32_t *) (BKPSRAM_BASE + 0x00) != CONFIGPARAMS_MAGIC)
        {
            // ConfigParams isn't initialized then write ConfigParams default values
            this->Reset();
        }
        else if ( (uint32_t)(*(__IO uint32_t *) (BKPSRAM_BASE + 0x04)) < (uint32_t)CONFIGPARAMS_VERSION)
        {
            if(this->Upgrade() < 0)
            {
                // Upgrade fail then reset ConfigParams
                this->Reset();
            }
        }
         */
    }

    void ConfigParams::Upgrade()
    {
        // Not implemented yet
        //return -1;
    }

    void ConfigParams::Reset()
    {
        /*
        *(__IO uint32_t *) (BKPSRAM_BASE + 0x00) = CONFIGPARAMS_MAGIC;
        *(__IO uint32_t *) (BKPSRAM_BASE + 0x04) = CONFIGPARAMS_VERSION;
        *(__IO uint32_t *) (BKPSRAM_BASE + 0x08) = 0x55555555;
        *(__IO uint32_t *) (BKPSRAM_BASE + 0x0C) = 0xAAAAAAAA;
        */

        /* TODO User default values */

        //return 0;
    }
    /*
    * - Write 32-bit data
    *       *(__IO uint32_t *) (BKPSRAM_BASE + 0x04) = Val;
    * - Read 32-bit data
    *       Val = *(__IO uint32_t *) (BKPSRAM_BASE + 0x04);
    */
}
