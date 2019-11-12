/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver for PIC32MX320F128H 
             Processor.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "Communication.h"    /*!< Communication definitions */

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst  - Transfer format (0 or 1).
 *                    Example: 0x0 - MSB first.
 *                             0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol  - SPI clock polarity (0 or 1).
 *                    Example: 0x0 - Idle state for clock is a low level; active
 *                                   state is a high level;
 *                             0x1 - Idle state for clock is a high level; active
 *                                   state is a low level.
 * @param clockEdg  - SPI clock edge (0 or 1).
 *                    Example: 0x0 - Serial output data changes on transition
 *                                   from idle clock state to active clock state;
 *                             0x1 - Serial output data changes on transition
 *                                   from active clock state to idle clock state.
 *
 * @return status   - Result of the initialization procedure.
 *                    Example:  0 - if initialization was successful;
 *                             -1 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
    SPI_CS_HIGH;
    SPI_CS_PIN_OUT;
	PORTB &= ~_BV(7); DDRB |= _BV(7); // PB7: SCK
	PORTB &= ~_BV(5); DDRB |= _BV(5); // PB5: MOSI
	DDRB |= _BV(6); // PB6: MISO
	
	/* SPI Master device, LSB first, Mode 3, 8MHz/16 = 500kHz */
	SPCR0 = ((1<<SPE) | (1<<MSTR) | (1<<SPR0)) | (1<<SPI2X);
	/* No interrupts or double speed... */
	SPSR0  = (1<<SPI2X);
	
	

    return 0;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data          - Data represents the write buffer.
 * @param bytesNumber   - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char* data,
                        unsigned char bytesNumber)
{
    unsigned char byte     = 0;
    unsigned char tempByte = 0;

    SPI_CS_LOW;
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SPDR0 = data[byte];
		loop_until_bit_is_set(SPSR0, SPIF);
    }
    SPI_CS_HIGH;

    return bytesNumber;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data          - Data represents the read buffer.
 * @param bytesNumber   - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
    unsigned char   byte            = 0;
    unsigned char   writeBuffer[4]  = {0, 0, 0, 0};

    for(byte = 0; byte < bytesNumber; byte++)
    {
        writeBuffer[byte] = data[byte];
    }
    if(slaveDeviceId == 1)
    {
        SPI_CS_LOW;
    }
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SPDR0 = writeBuffer[byte];
        loop_until_bit_is_set(SPSR0, SPIF);
        data[byte] = SPDR0;
    }
    loop_until_bit_is_set(SPSR0, SPIF);
    if(slaveDeviceId == 1)
    {
        SPI_CS_HIGH;
    }

    return bytesNumber;
}

/***************************************************************************//**
 * @brief Initializes the UART communication peripheral.
 *
 * @param baudRate - Baud rate value.
 *                   Example: 9600 - 9600 bps.
 *
 * @return status  - Result of the initialization procedure.
 *                   Example: -1 - if initialization was unsuccessful;
 *                             0 - if initialization was successful.
*******************************************************************************/
char UART_Init(unsigned long baudRate)
{
	uint16_t ubrr = F_CPU / 16 / baudRate - 1;
	
	/*Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN) | (1<<TXEN);
	/* Set frame format: 8data, 1stop bit */	
	UCSR1C = (1<<USBS) | (3<<UCSZ0);

    return 0;
}

/***************************************************************************//**
 * @brief Writes one character to UART.
 *
 * @param data - Character to write.
 *
 * @return None.
*******************************************************************************/
void UART_WriteChar(char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

/***************************************************************************//**
 * @brief Reads one character from UART.
 *
 * @param data - Read character.
 *
 * @return None.
*******************************************************************************/
void UART_ReadChar(char* data)
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	*data = UDR1;	
}

/***************************************************************************//**
 * @brief Writes one string of characters to UART.
 *
 * @param string - String of characters to write.
 *
 * @return None.
*******************************************************************************/
void UART_WriteString(const char* string)
{
    while(*string)
    {
        UART_WriteChar(*string++);
    }
}
