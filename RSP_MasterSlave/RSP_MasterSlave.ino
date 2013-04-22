//------------------------------------------------------------------------------
// Master Slave Demo for Rotary Stewart Platform
// dan@marginallycelver.com 2013-04-22
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.marginalyclever.com/ for more information.

#include <Servo.h>

// uncomment the next line to see more information in the serial output
//#define DEBUG_ON 1
#ifdef DEBUG_ON
#define DEBUG(x) { Serial.print(x); }
#else
#define DEBUG(x)
#endif


struct Arm {
  Servo servo;
  float angle;
  int flip;
  int angle_min;
  int angle_max;
};

Arm output[6];


int i;


void setup() {
  Serial.begin(9600);
  Serial.println(F("START"));
  
  for(i=0;i<6;++i) {
    output[i].servo.attach(7-i);
    output[i].servo.write(90);
    output[i].flip = 1;
  }
  
  output[1].flip ^= 1;  // servo 1 is acting backwards, reason unknown.
  
  configureLimits();
}


void loop() {
  for(i=0;i<6;++i) {
    // analogRead can only return a value from 0 to 1023.
    output[i].angle = analogRead(A0+i);
    // grow the max/min to make sure we never leave acceptable range.
    if(output[i].angle > output[i].angle_max) output[i].angle_max = output[i].angle;
    if(output[i].angle < output[i].angle_min) output[i].angle_min = output[i].angle;
    // make it range from 0 to 1
    output[i].angle = ( output[i].angle - output[i].angle_min ) / ( output[i].angle_max - output[i].angle_min );
    if(output[i].flip==1) output[i].angle = 1.0 - output[i].angle;

    DEBUG( output[i].angle );
    DEBUG('\t');
    
    // servo::writeMicroseconds takes a value from 1000...2000
    // servo::write takes a value from 0...180 (less precise)
    // tests showed that the inputs are rever
    output[i].servo.writeMicroseconds(1000.0 + 1000.0 * output[i].angle);
  }

  DEBUG('\n');
  delay(50);
}


void configureLimits() {
  // configure minimums
  Serial.println(F("Push the joystick all the way down and send \"MIN\"."));
  while(Serial.available()<3);
  i=0;
  while(Serial.available()>0) {
    Serial.read();
    ++i;
  }
  Serial.println(i);
  
  for(i=0;i<6;++i) {
    output[i].angle_min=analogRead(A0+i);
  }
  
  // configure maximums
  Serial.println(F("Push the joystick all the way down and send \"MAX\"."));
  while(Serial.available()<3);
  i=0;
  while(Serial.available()>0) {
    Serial.read();
    ++i;
  }
  Serial.println(i);
  
  for(i=0;i<6;++i) {
    output[i].angle_max=analogRead(A0+i);
  }
  
  Serial.println(F("Calibrated.  Enjoy!"));
}

//------------------------------------------------------------------------------
// Copyright (C) 2013 Dan Royer (dan@marginallyclever.com)
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

