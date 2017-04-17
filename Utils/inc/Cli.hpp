/**
 * @file    CLI.hpp
 * @author  Jeremy ROULLAND
 * @date    14 apr. 2017
 * @brief   Command Line Interface class
 */

#ifndef INC_CLI_HPP_
#define INC_CLI_HPP_

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef void (*FunctionFunc)();


/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
    /**
    * @class CLI
    * @brief
    *
    * HOWTO :
    *
    */

    class CLI
    {
    public:
        /**
         * @brief CLI default constructor
         */
        CLI();

        /**
         * @brief Add command
         */
        void AddCmd(char* cmd, FunctionFunc* f);

        void Start();

    protected:

    };
}

#endif /* INC_CLI_HPP_ */
