/**
 * @file    CLI.cpp
 * @author  Jeremy ROULLAND
 * @date    23 mar. 2017
 * @brief   Command Line Interface class
 */

#include "Cli.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CLI_BUF_MAX		40

/*----------------------------------------------------------------------------*/
/* Mapping                                                                    */
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/


namespace Utils
{
    CLI::CLI()
    {
    	//AddCmd("exit", NULL);
    }

    void CLI::AddCmd(char* cmd, FunctionFunc* f)
	{

	}

    void CLI::Start()
    {
    	char buf[CLI_BUF_MAX] = {0};
    	char c;
    	char *pch;
    	uint16_t i = 0;

    	putchar('>');

    	for( ;; )
    	{
    		c = getchar();

			if( ((c >= '0') && (c <= '9')) ||
				((c >= 'a') && (c <= 'z')) ||
				((c >= 'A') && (c <= 'Z')) ||
				((c == '.') ))
			{
	    		if(i < (CLI_BUF_MAX-2))
	    		{
		    		putchar(c);
					buf[i] = c;
					i++;
					buf[i] = '\0';
	    		}

			}
			else if(c == '\b')
			{
	    		putchar('\b');
    			putchar(' ');
	    		putchar('\b');
			}
			else if( (c == '\r') || (c == '\n') )
			{
        		pch = strtok (buf," ");

        		if(strcmp(pch,"exit") == 0)
        		{
        			return;
        		}
        		else if(strcmp(pch,"traces") == 0)
        		{
        			//TODO: Send signal to traces
        		}
        		else if(strcmp(pch,"golin") == 0)
        		{
					pch = strtok (NULL, " ");
					if(pch != NULL)
						float l = strtof(pch, NULL);
					else
						float l = 0.0;
        		}

				i=0;
				buf[i] = '\0';
		    	putchar('\r');
		    	putchar('\n');
		    	putchar('>');
			}

    	}

    }
}
