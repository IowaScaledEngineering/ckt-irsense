/*************************************************************************
Title:    IR reflective sensor v3.1
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
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
#include <avr/wdt.h>
#include <util/delay.h>

#define   TMD26711_ADDR   0x39

#define   ADC_THRESHOLD   0x300

// Debounce in 100ms increments
#define   ON_DEBOUNCE     2
#define   OFF_DEBOUNCE    2

#define   SDA   PB0
#define   SCL   PB2

#define   SDA_LOW   DDRB |= _BV(SDA); _delay_us(10);
#define   SDA_HIGH  DDRB &= ~_BV(SDA); _delay_us(10);
#define   SCL_LOW   PORTB &= ~_BV(SCL); _delay_us(10);
#define   SCL_HIGH  PORTB |= _BV(SCL); _delay_us(10);

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 78;  // 8MHz / 1024 / 78 = 100Hz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}

void i2cStart(void)
{
	SCL_HIGH;
	SDA_LOW;
	SCL_LOW;
	SDA_HIGH;
}

void i2cStop(void)
{
	SCL_LOW;
	SDA_LOW;
	SCL_HIGH;
	SDA_HIGH;
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack;

	do
	{
		if(byte & i)
		{
			SDA_HIGH;
		}
		else
		{
			SDA_LOW;
		}
		
		SCL_HIGH;
		SCL_LOW;
		
		i >>= 1;
	} while(i);
	;
	SDA_HIGH;  // Release SDA
	
	SCL_HIGH;
	if(PINB & _BV(SDA))
		ack = 0;
	else
		ack = 1;
	SCL_LOW;

	return ack;
}

uint8_t i2cReadByte(uint8_t ack)
{
	uint8_t i, data = 0;

	for(i=0; i<8; i++)
	{
		data = data << 1;
		SCL_HIGH;
		if(PINB & _BV(SDA))
			data |= 0x01;
		SCL_LOW;
	}
	
	if(ack)
		SDA_LOW
	SCL_HIGH;
	SCL_LOW;
	SDA_HIGH;

	return data;
}

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint16_t writeVal)
{
	uint8_t ack;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	ack = i2cWriteByte(writeVal);

	i2cStop();

	return ack;
}

uint16_t readWord(uint8_t addr, uint8_t cmd)
{
	uint16_t data;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);

	i2cStart();

	i2cWriteByte((addr << 1) | 0x01);
	data = i2cReadByte(1);
	data |= ((uint16_t)i2cReadByte(0) << 8);
	
	i2cStop();

	return data;
}

void init(void)
{
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();
	PORTB = _BV(SCL) | _BV(PB1);  // All outputs low except SCL and PB1
	DDRB |= _BV(PB1) | _BV(SCL) | _BV(PB3);
}

int main(void)
{
	uint16_t adc;
	uint8_t count = 0, detect = 0;
	
	// Application initialization
	init();
	initialize100HzTimer();
	sei();

	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(TMD26711_ADDR, 0x80|0x00, 0x00);  // Start with everything disabled
	writeByte(TMD26711_ADDR, 0x80|0x01, 0xFF);  // Minimum ATIME
	writeByte(TMD26711_ADDR, 0x80|0x02, 0xFF);  // Maximum integration time
	writeByte(TMD26711_ADDR, 0x80|0x03, 0xFF);  // Minimum wait time
	
	// Note: IRQ not currently used
	writeByte(TMD26711_ADDR, 0x80|0x08, 0x00);  // Set interrupt low threshold to 0x0000
	writeByte(TMD26711_ADDR, 0x80|0x09, 0x00);
	writeByte(TMD26711_ADDR, 0x80|0x0A, 0x00);  // Set interrupt low threshold to 0x0300
	writeByte(TMD26711_ADDR, 0x80|0x0B, 0x03);
	writeByte(TMD26711_ADDR, 0x80|0x0C, 0x10);  // Single out-of-range cycle triggers interrupt

	writeByte(TMD26711_ADDR, 0x80|0x0D, 0x00);  // Long wait disabled
	writeByte(TMD26711_ADDR, 0x80|0x0E, 0x08);  // Pulse count = 8 (default)
	writeByte(TMD26711_ADDR, 0x80|0x0F, 0x10);  // 100% LED drive strength, Use channel 0 diode

	writeByte(TMD26711_ADDR, 0x80|0x00, 0x27);  // Power ON, Enable proximity, Enable proximity interrupt (not used currently)

	while (1)
	{
		wdt_reset();

		if(decisecs >= 1)
		{
			decisecs = 0;

			adc = readWord(TMD26711_ADDR, 0x80|0x20|0x18);  // Read data register (0x80 = command, 0x20 = auto-increment)
	
			if(!detect & (adc >= ADC_THRESHOLD))
			{
				// ON debounce
				count++;
				if(count > ON_DEBOUNCE)
				{
					detect = 1;
					count = 0;
				}
			}
			else if(!detect & (adc < ADC_THRESHOLD))
			{
				count = 0;
			}

			else if(detect & (adc < ADC_THRESHOLD))
			{
				// OFF debounce
				count++;
				if(count > OFF_DEBOUNCE)
				{
					detect = 0;
					count = 0;
				}
			}
			else if(detect & (adc >= ADC_THRESHOLD))
			{
				count = 0;
			}
		}

		if(detect)
		{
			PORTB |= _BV(PB3);
			PORTB &= ~_BV(PB1);
		}
		else
		{
			PORTB &= ~_BV(PB3);
			PORTB |= _BV(PB1);
		}
	}
}



