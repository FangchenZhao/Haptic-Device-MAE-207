#include <math.h>

// Arduino pin numbers
#define sensorPosPin A2 // input pin for MR sensor

int updatedMR = 0;
double MR_Offset = 1390; //Loop offset of MR sensor
double T_angle = 0; //angle of top in body x-direction

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
}


void loop()
{
  int loop_a = 0; //servo ratation loop (CCW face down)
  int off = analogRead(sensorPosPin)+1018.5;
  if(off > 0){
    loop_a = 1;
    while(off > 0){
      off = off - MR_Offset;
      loop_a = loop_a + 1;
    }
  }
  else if(off < 0){
    while(off < 0){
      off = off + MR_Offset;
      loop_a = loop_a + 1;
    }
  }
  updatedMR = analogRead(sensorPosPin)+MR_Offset*loop_a;
//  Serial.print(updatedMR);
 T_angle = -0.259*updatedMR+96.2; // Compute the angle of the servo in degrees
//  Serial.print(T_angle);
}
