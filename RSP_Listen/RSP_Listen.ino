//------------------------------------------------------------------------------
// Test for Rotary Stewart Platform
// dan@marginallycelver.com 2013-03-11
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.marginalyclever.com/ for more information.


void setup() {
  Serial.begin(57600);
}


void loop() {  
  Serial.print(analogRead(0));  Serial.print('\t');
  Serial.print(analogRead(1));  Serial.print('\t');
  Serial.print(analogRead(2));  Serial.print('\t');
  Serial.print(analogRead(3));  Serial.print('\t');
  Serial.print(analogRead(4));  Serial.print('\t');
  Serial.print(analogRead(5));  Serial.print('\n');
  delay(100);
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

