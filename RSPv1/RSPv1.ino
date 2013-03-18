//------------------------------------------------------------------------------
// Rotary Stewart Platform
// dan@marginallycelver.com 2013-03-16
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform for more information.


//------------------------------------------------------------------------------
// for duemilanove atmega328 arduino board + easydriver stepper controller
// dan@marginallyclever.com 2011-06-21
//------------------------------------------------------------------------------
#include <Servo.h>
#include "Vector3.h"


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
#define TWOPI        (PI*2.0f)
#define DEG2RAD      (PI/180.0f)
#define RAD2DEG      (180.0f/PI)

#define BAUD    (57600)
#define MAX_BUF (64)

#define AXIS0   (2)
#define AXIS1   (3)
#define AXIS2   (4)
#define AXIS3   (5)
#define AXIS4   (6)
#define AXIS5   (7)


//------------------------------------------------------------------------------
struct RobotEffector  {
  Vector3 up;
  Vector3 left;
  Vector3 forward;

  Vector3 pos;
  Vector3 relative;

  Vector3 angle;
};


//------------------------------------------------------------------------------
struct RobotArm {
  Vector3 shoulder;
  Vector3 elbow;
  Vector3 elbow_relative;
  Vector3 wrist_pos;
  Vector3 wrist_relative;

  Servo s;
  float angle;
  int last_angle;
  // adjust for real world inaccuracies
  float scale;
  float zero;
};


//------------------------------------------------------------------------------
class Hexapod {
public:
  RobotArm arms[6];
  RobotEffector endeffector;
  float default_height;

public:
  void Setup();
};


//------------------------------------------------------------------------------
static float shoulder_to_elbow=1.6f;  // cm
static float elbow_to_wrist=8.8f;  // cm

// ee_to_w - end effector center to each wrist.
// Imagine a plane that passes through all wrist joints, centered between them.
// What is the X & Y on that plane to reach each a joint?
// The software will flip & rotate to figure out the other 5.
static float ee_to_w_y=0.4f;
static float ee_to_w_x=2.15f;

// c_to_s - base center to each shoulder (servo output gear).
// Imagine a plane that passes through all shoulder joints, centered between them.
// What is the X & Y on that plane to reach each a joint?
// The software will flip & rotate to figure out the other 5.
static float c_to_s_x=4.14f;
static float c_to_s_y=1.28f;

Hexapod hexapod;

char buffer[MAX_BUF];
int sofar;


//------------------------------------------------------------------------------
void Hexapod::Setup() {
  int i;
  float aa,bb,cc;
  Vector3 temp,n,ortho,t1,t2,t3;

  for(i=0;i<3;++i) {
    RobotArm &arma=this->arms[i*2+0];
    RobotArm &armb=this->arms[i*2+1];

    // find wrist position
    n.x=cos(i*TWOPI/3.0f);
    n.y=sin(i*TWOPI/3.0f);
    n.z=0;
    ortho.x=-n.y;
    ortho.y=n.x;
    ortho.z=0;
    arma.wrist_pos = n*ee_to_w_x - ortho*ee_to_w_y;
    armb.wrist_pos = n*ee_to_w_x + ortho*ee_to_w_y;
    arma.shoulder = n*c_to_s_x - ortho*c_to_s_y;
    armb.shoulder = n*c_to_s_x + ortho*c_to_s_y;
    arma.elbow = n*c_to_s_x - ortho*(c_to_s_y+shoulder_to_elbow);
    armb.elbow = n*c_to_s_x + ortho*(c_to_s_y+shoulder_to_elbow);
    
    arma.wrist_relative=arma.wrist_pos;
    armb.wrist_relative=armb.wrist_pos;
    
    // the tricky part is getting wrist_pos.z
    aa=(arma.elbow-arma.wrist_pos).Length();
    cc=elbow_to_wrist;
    bb=sqrt((cc*cc)-(aa*aa));
    arma.wrist_pos.z=bb;
    armb.wrist_pos.z=bb;

    arma.elbow_relative=arma.elbow-arma.shoulder;
    armb.elbow_relative=armb.elbow-armb.shoulder;
  }
  this->default_height=bb;
  this->endeffector.pos.z=0;
  this->endeffector.up.Set(0,0,1);
  this->endeffector.forward.Set(1,0,0);
  this->endeffector.left.Set(0,1,0);
  this->endeffector.angle.Set(0,0,0);
  
  this->arms[0].s.attach(AXIS0);
  this->arms[1].s.attach(AXIS1);
  this->arms[2].s.attach(AXIS2);
  this->arms[3].s.attach(AXIS3);
  this->arms[4].s.attach(AXIS4);
  this->arms[5].s.attach(AXIS5);
  this->arms[0].scale=-1;
  this->arms[1].scale=1;
  this->arms[2].scale=-1;
  this->arms[3].scale=1;
  this->arms[4].scale=-1;
  this->arms[5].scale=1;
}


//------------------------------------------------------------------------------
void line(float x,float y,float z,float a,float b,float c) {
  Vector3 axis;
  Vector3 target_pos(x,y,z);
  Vector3 target_ang(a,b,c);
  Vector3 start_pos = hexapod.endeffector.pos;
  Vector3 start_ang = hexapod.endeffector.angle;
  Vector3 da, dp, ortho, w, wop, temp, p1, p2,n,r;
  float r0,r1,d,hh,dt;
  int i;
  
  target_pos.z+=hexapod.default_height;
  start_pos.z+=hexapod.default_height;

  // @TODO: measure distance from start to destination
  // @TODO: divide by feed rate to get travel time
  float travel_time = 1.0;

  Serial.print(F("START\n"));

  float start=millis()*0.001;
  do {
    // Interpolate over travel time to move end effector
    float now = millis()*0.001;
    dt = ( now - start ) / travel_time;
    if(dt > 1) dt = 1;

    //Serial.print(now);
    //Serial.print(F(" - "));
    //Serial.print(start);
    //Serial.print(F(" = "));
    Serial.print(dt);
    Serial.print(F("\t"));

    hexapod.endeffector.pos = ( target_pos - start_pos ) * dt + start_pos;

    // roll pitch & yaw
    da = ( target_ang - start_ang ) * dt + start_ang;
    da *= DEG2RAD;
    
    hexapod.endeffector.up.Set(0,0,1);
    hexapod.endeffector.forward.Set(1,0,0);
    hexapod.endeffector.left.Set(0,1,0);

    // roll
    axis.Set(1,0,0);
    hexapod.endeffector.up.Rotate(axis,da.x);
    hexapod.endeffector.forward.Rotate(axis,da.x);
    hexapod.endeffector.left.Rotate(axis,da.x);

    // pitch
    axis.Set(0,1,0);
    hexapod.endeffector.up.Rotate(axis,da.y);
    hexapod.endeffector.forward.Rotate(axis,da.y);
    hexapod.endeffector.left.Rotate(axis,da.y);

    // yaw
    axis.Set(0,0,1);
    hexapod.endeffector.up.Rotate(axis,da.z);
    hexapod.endeffector.forward.Rotate(axis,da.z);
    hexapod.endeffector.left.Rotate(axis,da.z);
  
    
    // update wrist positions
    RobotEffector &ee = hexapod.endeffector;
    for(i=0;i<6;++i) {
      RobotArm &arm = hexapod.arms[i];
      arm.wrist_pos = ee.pos + 
                      ee.forward * arm.wrist_relative.x +
                      ee.left    * arm.wrist_relative.y + 
                      ee.up      * arm.wrist_relative.z;
    }

    // find shoulder angles & elbow positions
    for(i=0;i<6;++i) {
      RobotArm &arm=hexapod.arms[i];
      
      // get wrist position on plane of bicep
      // @TODO: calculate once in setup
      ortho.x=cos((i>>1)*TWOPI/3.0f);
      ortho.y=sin((i>>1)*TWOPI/3.0f);
      ortho.z=0;

      w = arm.wrist_pos - arm.shoulder;
      
      a=w | ortho;  //endeffector' distance
      wop = w - ortho * a;
      //arm.wop.pos=wop + arm.shoulder;  // e' location
      //vector_add(arm.wop,wop,arm.shoulder);

      // because wop is projected onto the bicep plane, wop-elbow is not the same as wrist-elbow.
      // we need to find wop-elbow before we can calculate the angle at the shoulder.
      c=elbow_to_wrist;
      b=sqrt(c*c-a*a);  // e'j distance

      // use intersection of circles to find elbow point.
      //a = (r0r0 - r1r1 + d*d ) / (2 d) 
      r1=b;  // circle 1 centers on e'
      r0=shoulder_to_elbow;  // circle 0 centers on shoulder
      d=wop.Length();
      // distance from shoulder to the midpoint between the two possible intersections
      a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
      // find the midpoint
      n=wop;
      n.Normalize();
      
      temp=arm.shoulder+(n*a);

      // with a and r0 we can find h, the distance from midpoint to intersections.
      hh=sqrt(r0*r0-a*a);
      
      // get a normal to the line wop in the plane orthogonal to ortho
      r = ortho ^ n;
      p1 = temp + r * hh;
      p2 = temp - r * hh;
      
      if(i%2==0) arm.elbow=p1;
      else       arm.elbow=p2;

      // use atan2 to find theta
      temp=arm.elbow-arm.shoulder;
      y=temp.z;
      temp.z=0;
      x=temp.Length();
      
      if( ( arm.elbow_relative | temp ) < 0 ) x=-x;
      
      arm.angle=atan2(-y,x) * RAD2DEG;

      if(arm.angle<-90) arm.angle=-90;
      if(arm.angle> 90) arm.angle= 90;
      
      // we don't care about elbow angle.  we could find it here if we needed it.

      // update servo
      Serial.print(arm.angle);
      Serial.print(F("\t"));
      
      arm.angle = (arm.angle * arm.scale) * (500.0f/90.0f) + 1500.0f;
      if(arm.last_angle!=arm.angle) {
        arm.s.writeMicroseconds(arm.angle);
        arm.last_angle=arm.angle;
      }
    }
    Serial.print(F("\n"));
  } while( dt < 1 );
  
  hexapod.endeffector.pos = target_pos;
  hexapod.endeffector.pos.z -= hexapod.default_height;

  hexapod.endeffector.angle = target_ang;
  
  Serial.print(F("END\n"));
}


//------------------------------------------------------------------------------
void processCommand() {
  if(!strncmp(buffer,"M114",4)) {
    Serial.print(hexapod.endeffector.pos.x);       Serial.print(F(", "));
    Serial.print(hexapod.endeffector.pos.y);       Serial.print(F(", "));
    Serial.print(hexapod.endeffector.pos.z);       Serial.print(F(" - "));
    Serial.print(hexapod.endeffector.angle.x);     Serial.print(F(", "));
    Serial.print(hexapod.endeffector.angle.y);     Serial.print(F(", "));
    Serial.println(hexapod.endeffector.angle.z);
  } else if(!strncmp(buffer,"G00",3) || !strncmp(buffer,"G01",3)) {
    float xx=hexapod.endeffector.pos.x;
    float yy=hexapod.endeffector.pos.y;
    float zz=hexapod.endeffector.pos.z;
    float aa=hexapod.endeffector.angle.x;
    float bb=hexapod.endeffector.angle.y;
    float cc=hexapod.endeffector.angle.z;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'x': case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'y': case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'z': case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      case 'a': case 'A': aa=atof(ptr+1);  Serial.print('a'); Serial.println(aa); break;
      case 'b': case 'B': bb=atof(ptr+1);  Serial.print('b'); Serial.println(bb); break;
      case 'c': case 'C': cc=atof(ptr+1);  Serial.print('c'); Serial.println(cc); break;
      default: ptr=0; break;
      }
    }
    
    line(xx,yy,zz,aa,bb,cc);
  }
}


//------------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  Serial.println(F("Hello, World!  I am a Rotary Stewart Platform."));
  
  hexapod.Setup();
  //line(0,0,0,0,0,0);

  // setup receiving buffer
  sofar=0;
}


//------------------------------------------------------------------------------
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
  }

  if(sofar>0 && buffer[sofar-1]==';') {
    // what if message fails/garbled?
    
    // echo confirmation
    buffer[sofar]=0;
    Serial.println(buffer);

    processCommand();
    
    sofar=0;
    
    // echo completion
    Serial.println("Done.");  // @TODO: improve.
  }
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

