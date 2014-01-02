/*************************************************************************
Title:    IR reflective sensor
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define OCR1A_VALUE  0xC7
volatile uint8_t pulsesRemaining = 0;

void initialize38kHzTimer(void)
{
	TCCR1 = _BV(CTC1) | _BV(CS10);
	OCR1C = OCR1A = OCR1A_VALUE; // 38kHz at 8MHz
	OCR1B = OCR1A_VALUE/2; // 
	GTCCR = 0;
	TIMSK &= ~_BV(TOIE1);
	TIMSK |= _BV(OCIE1A) | _BV(OCIE1B);	
}

ISR(TIMER1_COMPA_vect)
{
	PORTB &= ~_BV(PB1);
	if (pulsesRemaining)
		pulsesRemaining--;
}

ISR(TIMER1_COMPB_vect)
{
	if (pulsesRemaining)
		PORTB |= _BV(PB1);
}


void init(void)
{
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();
	DDRB |= _BV(PB1);
	
}


int main(void)
{
	// Application initialization
	init();
	initialize38kHzTimer();
	sei();	

	while (1)
	{
		wdt_reset();

		pulsesRemaining = 24;

		_delay_ms(50);



	}
}



