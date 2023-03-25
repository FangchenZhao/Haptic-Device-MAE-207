#include <math.h>

// Arduino pin numbers
#define sensorPosPin A2 // input pin for MR sensor

int updatedMR = 0;
double MR_Offset = 980; //Offset of MR sensor
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
 if (analogRead(sensorPosPin)<400){
    updatedMR = analogRead(sensorPosPin);
 }
  else {
    updatedMR = analogRead(sensorPosPin)-MR_Offset;
  }
//  Serial.print(updatedMR);
 T_angle = -0.259*updatedMR+96.2; // Compute the angle of the servo in degrees
//  Serial.print(T_angle);
}
