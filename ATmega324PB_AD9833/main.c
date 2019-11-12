/*
 * ATmega324PB_AD9833.c
 *
 * Created: 2019/11/10 21:02:34
 * Author : Pluto
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "Console.h"
#include "ad9833.h"

/***************************************************************************//**
 * @brief Main function.
 *
 * @return None.
*******************************************************************************/
int main(void)
{
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
    AD9833_SetFrequency(AD9833_REG_FREQ0, 0x10C7);	// 400 Hz, 0x10C7 =  400 * (268.435456/25)
    AD9833_SetFrequency(AD9833_REG_FREQ1, 0x418958);// 400 kHz
    AD9833_SetPhase(AD9833_REG_PHASE0,0x00);
    AD9833_SetPhase(AD9833_REG_PHASE1,0x00);
    AD9833_ClearReset();
	AD9833_Setup(AD9833_FSEL0, AD9833_PSEL1, AD9833_OUT_MSB);
		
	while(1);
#if 0
	while(1)
	{
		CONSOLE_Print(4,25, "Sinusoid  ");
		CONSOLE_Print(5, 30, "400 Hz  ");
		AD9833_Setup(AD9833_FSEL0, AD9833_PSEL1, AD9833_OUT_SINUS);
		_delay_ms(1000);
		CONSOLE_Print(4, 25, "Sinusoid  ");
		CONSOLE_Print(5, 30, "400 kHz   ");
		AD9833_Setup(AD9833_FSEL1, AD9833_PSEL1, AD9833_OUT_SINUS);
		_delay_ms(1000);
		CONSOLE_Print(4,25, "Triangle  ");
		CONSOLE_Print(5, 30, "400 Hz  ");
		AD9833_Setup(AD9833_FSEL0, AD9833_PSEL1, AD9833_OUT_TRIANGLE);
		_delay_ms(1000);
		CONSOLE_Print(4,25, "Triangle  ");
		CONSOLE_Print(5, 30, "400 kHz  ");
		AD9833_Setup(AD9833_FSEL1, AD9833_PSEL1, AD9833_OUT_TRIANGLE);
		_delay_ms(1000);
		CONSOLE_Print(4,25, "Square    ");
		CONSOLE_Print(5, 30, "400 Hz  ");
		AD9833_Setup(AD9833_FSEL0, AD9833_PSEL1, AD9833_OUT_MSB);
		_delay_ms(1000);
		CONSOLE_Print(4,25, "Square    ");
		CONSOLE_Print(5, 30, "400 kHz  ");
		AD9833_Setup(AD9833_FSEL1, AD9833_PSEL1, AD9833_OUT_MSB);
		_delay_ms(1000);
	}
#endif	
}
