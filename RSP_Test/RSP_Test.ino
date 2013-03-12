//------------------------------------------------------------------------------
// Test for Rotary Stewart Platform
// dan@marginallycelver.com 2013-03-11
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.marginalyclever.com/ for more information.


#include <Servo.h>


Servo arm[6];


void setup() {
  Serial.begin(57600);
  Serial.println(F("Hello, World!"));

  int i;
  for(i=0;i<6;++i) {
    arm[i].attach(2+i);
  }
}


// reverse the direction for every second servo.
void set_servo(int index,int angle) {
  int j = 90 + angle * ((index%2)?1:-1);
  arm[index].write( j );
}


void loop() {
  float t=millis()*0.001;

  int i,j;
  for(i=0;i<6;++i) {
    j = sin(t+(float)i*PI/3.0)*30;
    set_servo(i,j);
    Serial.print(j);
    Serial.print('\t');
  }
  
  Serial.print('\n');
  delay(50);
}



//------------------------------------------------------------------------------
// Copyright (C) 2012 Dan Royer (dan@marginallyclever.com)
// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------

