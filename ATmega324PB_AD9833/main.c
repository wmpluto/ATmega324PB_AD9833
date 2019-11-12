/*
 * ATmega324PB_AD9833.c
 *
 * Created: 2019/11/10 21:02:34
 * Author : Pluto
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "Console.h"
#include "Command.h"
#include "ad9833.h"

/***************************************************************************//**
 * @brief Main function.
 *
 * @return None.
*******************************************************************************/
int main(void)
{
	char   receivedCmd[20];
	double param[5];
	char   cmd        = 0;
	char   paramNo    = 0;
	char   cmdType    = -1;
	char   invalidCmd = 0;

	/* Clock Prescaler Select */
	CLKPR = 0x80;
	CLKPR = 0x00;	

	/*!< Initialize the console. */
	CONSOLE_Init(19200);
	CONSOLE_Print("\r\n\r\nATA9833 Test Code\r\n\r\n");

    if(AD9833_Init())
	    CONSOLE_Print("AD9833 OK\r\n");
    else
	    CONSOLE_Print("AD9833 Err\r\n");
    
    AD9833_Reset();
    AD9833_SetFrequency(AD9833_REG_FREQ0, 0x10C7);	// 400 Hz
    AD9833_SetFrequency(AD9833_REG_FREQ1, 0x418958);// 400 kHz
    AD9833_SetPhase(AD9833_REG_PHASE0,0x00);
    AD9833_SetPhase(AD9833_REG_PHASE1,0x00);
    AD9833_ClearReset();
    CONSOLE_Print("Output signal: ");	
	AD9833_Setup(AD9833_FSEL0, AD9833_PSEL1, AD9833_OUT_MSB);
	
	
	while(1);
	
#if 0	
	while(1)
	{
		/*!< Read the command entered by user through UART. */
		CONSOLE_GetCommand(receivedCmd);
		
		invalidCmd = 0;
		for(cmd = 0; cmd < cmdNo; cmd++)
		{
			paramNo = 0;
			cmdType = CONSOLE_CheckCommands(receivedCmd, cmdList[cmd], param, &paramNo);
			if(cmdType == UNKNOWN_CMD)
			{
				invalidCmd++;
			}
			else
			{
				cmdFunctions[cmd](param, paramNo);
			}
		}
		/*!< Send feedback to user, if the command entered by user is not a valid one. */
		if(invalidCmd == cmdNo)
		{
			CONSOLE_Print("Invalid command!\r\n");
		}
	}
#endif
}
