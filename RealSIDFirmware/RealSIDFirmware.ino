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

char newsid[25];
char oldsid[25];
bool dataready = false;
char buffer[50];
int idx = 0;
unsigned long lastupdate = 0;
SID SIDchip;

void setup()
{
  Serial.begin(115200);
  SIDchip.Reset();
}

char decode(char ch)
{
  if (ch >= 'A')
    return ch - 55;
  else
    return ch - 48;
}

void updateSID()
{
  for (int i = 0; i < 25; i++)
  {
    if (oldsid[i] != newsid[i])
    {
      oldsid[i] = newsid[i];
      SIDchip.Poke(i, newsid[i]);
    }
  }
}

void readData()
{
  if (Serial.available() > 0)
  {
      char ch = Serial.read();
      if (ch == '!')
      {
        if (idx == 50)
        {
          for (int i = 0; i < 25; i++)
          {
            char highnibble = buffer[i * 2];
            char lownibble = buffer[i * 2 + 1];
            newsid[i] = (decode(highnibble) << 4) | decode(lownibble);
          }
          dataready = true;
        }
        idx = 0;
      }
      else
      {
        if (idx < 50)
          buffer[idx] = ch;
        idx++;
      }
  }
}

void loop()
{
  if (!dataready)
    readData();
  
  // Check time and update SID if needed
  unsigned long timenow = millis();
  if (timenow - lastupdate >= 20)
  {
    lastupdate = timenow;
    if (dataready == true)
    {
      dataready = false;
      updateSID();
    }
  
    // Send more data!
    Serial.write('?');
    Serial.flush();
  }
}
