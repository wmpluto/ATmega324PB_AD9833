/***************************************************************************//**
 *   @file   AD9833.c
 *   @brief  Implementation of AD9833 Driver.
 *   @author Mihai Bancisor
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 537
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD9833.h"		// AD9833 definitions.

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral and resets the part.
 *
 * @return 1.
*******************************************************************************/
unsigned char AD9833_Init(void)
{
    SPI_Init(0, 1000000, 1, 1);
    AD9833_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
	
    return (1);
}

/***************************************************************************//**
 * @brief Sets the Reset bit of the AD9833.
 *
 * @return None.
*******************************************************************************/
void AD9833_Reset(void)
{
    AD9833_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
}

/***************************************************************************//**
 * @brief Clears the Reset bit of the AD9833.
 *
 * @return None.
*******************************************************************************/
void AD9833_ClearReset(void)
{
	AD9833_SetRegisterValue(AD9833_REG_CMD);
}
/***************************************************************************//**
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value to write to the register.
 *
 * @return  None.    
*******************************************************************************/
void AD9833_SetRegisterValue(unsigned short regValue)
{
	unsigned char data[5] = {0x03, 0x00, 0x00};	
	
	data[1] = (unsigned char)((regValue & 0xFF00) >> 8);
	data[2] = (unsigned char)((regValue & 0x00FF) >> 0);
	SPI_CS_LOW;	    
	SPI_Write(data+1, 2);
	SPI_CS_HIGH;
}

/***************************************************************************//**
 * @brief Writes to the frequency registers.
 *
 * @param -  reg - Frequence register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
void AD9833_SetFrequency(unsigned short reg, unsigned long val)
{
	unsigned short freqHi = reg;
	unsigned short freqLo = reg;
	
	freqHi |= (val & 0xFFFC000) >> 14 ;
	freqLo |= (val & 0x3FFF);
	AD9833_SetRegisterValue(AD9833_B28);
	AD9833_SetRegisterValue(freqLo);
	AD9833_SetRegisterValue(freqHi);
}
/***************************************************************************//**
 * @brief Writes to the phase registers.
 *
 * @param -  reg - Phase register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
void AD9833_SetPhase(unsigned short reg, unsigned short val)
{
	unsigned short phase = reg;
	phase |= val;
	AD9833_SetRegisterValue(phase);
}
/***************************************************************************//**
 * @brief Selects the Frequency,Phase and Waveform type.
 *
 * @param -  freq  - Frequency register used.
 * @param -  phase - Phase register used.
 * @param -  type  - Type of waveform to be output.
 *
 * @return  None.    
*******************************************************************************/
void AD9833_Setup(unsigned short freq,
				  unsigned short phase,
			 	  unsigned short type)
{
	unsigned short val = 0;
	
	val = freq | phase | type;
	AD9833_SetRegisterValue(val);
}
/***************************************************************************//**
 * @brief Sets the type of waveform to be output.
 *
 * @param -  type - type of waveform to be output.
 *
 * @return  None.    
*******************************************************************************/
void AD9833_SetWave(unsigned short type)
{
	AD9833_SetRegisterValue(type);
}