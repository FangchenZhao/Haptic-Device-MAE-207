#include <util/atomic.h>
#include <math.h>
#include <Servo.h>
Servo myservo;  // create servo object to control a servo

// Arduino pin numbers
#define ENCA 3 // Encoder A input YELLOW
#define ENCB 2  // Encoder B input WHITE
#define sensorPosPin A2 // input pin for MR sensor
#define SW_pin 13 // digital pin connected to switch output orange
#define X_pin A4 // analog pin connected to X output green
#define Y_pin A5 // analog pin connected to Y output blue

#define PWM 7 // PWM output pin for motor
#define IN1 6 // Motor driver output 1 for motor
#define IN2 5 // Motor driver output 2 for motor
#define servo_pin A0  // speed output pin for servo


int updatedMR = 0;     // keeps track of the latest updated value of the MR sensor reading
double B_angle = 0; //angle of bottom
double T_angle = 0; //angle of top
double Joystick_h = 0; //Joystick Case hold
int Joystick = 0; //Joystick Case LOW (pressed)
double Plante = 0;

int pos = 0;
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

double A_velocity = 0; //Angular velocity
double B_angle_old = 0;
double A_velocity_old = 0; 
double time_old = 0;

double Joystick_old = 0;
double Plante_old = 0;

void setup() {
  Serial.begin(115200);
  
  // Input pins
  pinMode(ENCA,INPUT); // Encoder A input
  pinMode(ENCB,INPUT); // Encoder B input
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(SW_pin, INPUT); // set Joystick pin to be an input
  pinMode(X_pin, INPUT); // set Joystick pin to be an input
  pinMode(Y_pin, INPUT); // set Joystick pin to be an input

  // Output pins
  pinMode(PWM, OUTPUT);  // PWM pin for motor
  pinMode(IN1,OUTPUT);  // Motor driver output 1 for motor
  pinMode(IN2,OUTPUT);  // Motor driver output 2 for motor

  myservo.attach(servo_pin);
}

void loop() {
// --------------------------------------------------------------
// Angular position of the bottom system (Motor Encoder)
// -------------------------------------------------------------- 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  B_angle = 0.174*pos+3.87;
//  Serial.println(B_angle);
  
// --------------------------------------------------------------
// Angular position of the top system (Magnetoresistive (MR) Sensors)
// --------------------------------------------------------------
 if (analogRead(sensorPosPin)<400){
    updatedMR = analogRead(sensorPosPin);
 }
  else {
    updatedMR = analogRead(sensorPosPin)-955;
  }
 //Serial.print(updatedMR);
 T_angle = -0.259*updatedMR+96.2; // Compute the angle of the servo in degrees
//  Serial.print(T_angle);

// --------------------------------------------------------------
// Velocity of the plante
// --------------------------------------------------------------
  double time_new = millis();
  double td = time_new-time_old; //[s] time difference between two loops
  time_old = time_new;
  double a = 0.8;
  double A_velocity_B = (B_angle-B_angle_old)/td; //[/s] Angular velocity before filter
  double A_velocity = a*A_velocity_B+(1-a)*A_velocity_old; //[/s] filted angular velocity by infinite impulse response filter
  double R_p = sin(T_angle); //Radius of the plante
  B_angle_old = B_angle; 
  A_velocity_old = A_velocity; 
  double Velocity = A_velocity*R_p; 
//  Serial.print(Velocity)
  
// --------------------------------------------------------------
// ON/OFF of the system (Push Joystick)
// --------------------------------------------------------------
  if(digitalRead(SW_pin) == HIGH) {
    Joystick_h = 0;
  }

if(Joystick_h == 0){
  if(digitalRead(SW_pin) == LOW && Joystick == 0) {
//  Serial.print("ON");
//  Serial.print(" | ");
  Joystick = 1;
    if (Joystick == Joystick_old){
    }
    else {
    Serial.print("Universe ON");
    Joystick_h = 1;
    }
 } else if (digitalRead(SW_pin) == LOW && Joystick == 1) {
//  Serial.print("OFF");
//  Serial.print(" | ");
  Joystick = 0;
    if (Joystick == Joystick_old){
    }
    else {
    Serial.print("Universe OFF");
    Joystick_h = 1;
    }
 }
}

//  Serial.print("Joystick: ");
//  Serial.print(Joystick);

// --------------------------------------------------------------
// Modes of the system (Move Joystick)
// --------------------------------------------------------------
//  Serial.print("X-axis: ");
//  Serial.print(analogRead(X_pin));
//  Serial.print(" | ");
//  Serial.print("Y-axis: ");
//  Serial.print(analogRead(Y_pin));
//  Serial.println(" | ");
//  Serial.println();
  if(Joystick == 0){
    Plante = 0;
  }
  else if(Joystick == 1){ 
  if(analogRead(X_pin)<300 || analogRead(Y_pin)<300){
    if(analogRead(X_pin)<analogRead(Y_pin)){
      Plante = 1;
    }
    else if(analogRead(X_pin)>analogRead(Y_pin)){
      Plante = 2;
    }
  }
  else if(analogRead(X_pin)>700 || analogRead(Y_pin)>700){
    if (analogRead(X_pin)>analogRead(Y_pin)){
      Plante = 3;
    }
    else if(analogRead(X_pin)<analogRead(Y_pin)){
      Plante = 4;
    }
  }
  }

// --------------------------------------------------------------
// Plantes Rendering and printing
// -------------------------------------------------------------- 
  if(Plante == 0){
    if (Plante == Plante_old){
      if (Joystick == Joystick_old){
      }
      else {
//      Serial.print("Empty Universe.");
      Serial.println();
      }
    }
    else {
//    Serial.print("Empty Universe.");
    Serial.println();
    }
    myservo.write(90);
    setMotor(0,0,PWM,IN1,IN2);
  }
  
  else if(Plante == 1){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Mercury!");
    Serial.println();
    }
    if (T_angle > 60){
    myservo.write(80);
    }
    else if (T_angle < 20){
    myservo.write(103);
    }
    else {
    myservo.write(100);
    }
    setMotor(1,255,PWM,IN1,IN2);
  }
  
  else if(Plante == 2){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Venus!");
    Serial.println();
    }
    if (T_angle > 68){
    myservo.write(80);
    }
    else if (T_angle < 28){
    myservo.write(108);
    }
    else {
    myservo.write(105);
    }
    setMotor(1,200,PWM,IN1,IN2);
  }
  
  else if(Plante == 3){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Earth!");
    Serial.println();
    }
    if (T_angle > 80){
    myservo.write(80);
    }
    else if (T_angle < 40){
    myservo.write(112);
    }
    else {
    myservo.write(109);
    }
    setMotor(1,180,PWM,IN1,IN2);
  }
  
  else if(Plante == 4){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Mars!");
    Serial.println();
    }
    if (T_angle > 95){
    myservo.write(80);
    }
    else if (T_angle < 55){
    myservo.write(108);
    }
    else {
    myservo.write(105);
    }
    setMotor(1,150,PWM,IN1,IN2);
  }

Joystick_old = Joystick;
Plante_old = Plante;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
