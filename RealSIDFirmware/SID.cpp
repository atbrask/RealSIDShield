/*
RealSIDFirmware
Version 1.0

Copyright (c) 2013, A.T.Brask (atbrask[at]gmail[dot]com)
All rights reserved,

Demonstration code for the RealSIDShield Arduino shield.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "SID.h"

void SID::Reset() 
{
  char addr;
  
  // Set up 74574 clock
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  
  // Set up SID chip select
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

  // Copied from the SIDaster project:
  // 1MHz generation on OC1A - Clk 16 MHz - set pin 10 as OC1A output
  // Reset settings of Timer/Counter register 1
  // Set compare match output A to toogle
  // Set waveform generation mode to CTC (Clear Counter on Match)
  // Set clock select to clock/1024 (from prescaler)
  // Set output compare register A to 8 (i.e. OC1A Toggle every 7+1=8 Clk pulses)
  pinMode(10, OUTPUT);
  TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0) | (1<<WGM11) | (1<<WGM10));
  TCCR1B &= ~((1<<WGM13) | (1<<WGM12) | (1<<CS12) | (1<<CS11) | (1<<CS10));
  TCCR1A |= (0<<COM1B1) | (1<<COM1B0);
  TCCR1A |= (0<<WGM11) | (0<<WGM10);
  TCCR1B |= (0<<WGM13) | (1<<WGM12);
  TCCR1B |= (0<<CS12) | (0<<CS11) | (1<<CS10);
  OCR1A = 7;

  // Reset SID
  delayMicroseconds(20000);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delayMicroseconds(20000);
  digitalWrite(13, HIGH);
  delayMicroseconds(20000);

  // Reset SID registers (0..24 are write-only and 25...28 are read-only)
  for(addr = 0; addr < 25; addr++)
    Poke(addr, 0);
}

void SID::Poke(char addr, char value)
{
  // Set databus in output mode
  DDRD |= 0b11111110;
  DDRB |= 0b00111111;
  
  // Disable SID
  PORTB |= 0b00001000;

  // Address bits 0 and 1 goes to B0 and B1 and bits 2..4 goes to D2...D4. The write-bit goes implicitly to D5.
  PORTB = (PORTB & 0b11111100) | (addr & 0b00000011);
  PORTD = (PORTD & 0b11000011) | (addr & 0b00011100);

  // Pulse the 74374's clock
  PORTB |= 0b00010000;
  PORTB &= 0b11101111;
  
  // Put the value on the databus
  PORTB = (PORTB & 0b11111100) | (value & 0b00000011);
  PORTD = (PORTD & 0b00000011) | (value & 0b11111100);
  
  // Enable SID
  PORTB &= 0b11110111;
}

char SID::Peek(char addr)
{
  // Set databus in output mode
  DDRD |= 0b11111110;
  DDRB |= 0b00111111;
  
  // Disable SID
  PORTB |= 0b00001000;

  // Put address and R/W bit on databus
  // Address bits 0 and 1 goes to B0 and B1 and bits 2..4 goes to D2...D4. The read-bit goes to D5.
  PORTB = (PORTB & 0b11111100) | (addr & 0b00000011);
  PORTD = (PORTD & 0b11000011) | (addr & 0b00011100) | 0b00010000;
  
  // Pulse the 74374's clock
  PORTB |= 0b00010000;
  PORTB &= 0b11101111;

  // Set databus in input mode
  DDRD &= 0b00000011;
  DDRB &= 0b11111100;

  // Enable SID and wait one clock cycle (more or less)
  PORTB &= 0b11110111;
  delayMicroseconds(1);
  
  // Read databus and return
  return (PINB & 0b00000011) | (PIND & 0b11111100);
}
