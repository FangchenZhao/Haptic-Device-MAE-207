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
double MR_Offset = 1390; //Loop offset of MR sensor
double B_angle = 0; //angle of bottom in space y-direction
double T_angle = 0; //angle of top in body x-direction

// Joystick Control
double Joystick_h = 0; //Joystick Case hold
int Joystick = 0; //Joystick Case LOW (pressed)
double Plante = 0;
double Joystick_old = 0;
double Plante_old = 0;

// Servo Control
int S_speed = 0; //0-180 (0:CCW max speed, 90:Stop, 180:CW max speed)
double angle_tolerance = 20; //angle tolerance (so large because of the wobble servo)
double weight_effect_speed = 0; //
int Real_angle = 0; //
int Real_Distance = 0; //mm

// Motor Control
int pos = 0;
int target = 0;
int speed_ratio = 0; //
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int pos_old = 0;

// Calculation Variables
double time_old = 0;
double A_velocity = 0; //Angular velocity
double B_angle_old = 0;
double A_velocity_old = 0; 

// User Input Data
int Modle_P_weight = 0; //kg
double model_arm_length = 12*25.4; //mm
double Model_ratio_d = 1.3368*pow(10,-6); //mm/km (model to real object [orbital distance])
double Model_ratio_r = 5.20598*pow(10,-3); //mm/km (model to real object [radius])
double material_density = 1000; //kg/m^3 (60% PLA)
double gear_ratio = 25; //Gearbox and belt gear
double G = 6.67428*pow(10,-20); //km^3 kg^-1 s^-2
double M_C = 1.98892*pow(10,30); //kg (mass of center [sun])

int Plante_Diameter = 0; //km
int Plante_Weight = 0; //kg
int Plante_Density = 0; //kg/m^2
int Plante_Distance = 0; //km
int Orbital_Period = 0; //days
int Orbital_Velocity = 0; //km/s

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
  Joystick = 1;
    if (Joystick == Joystick_old){
    }
    else {
    Serial.print("Universe ON");
    Joystick_h = 1;
    }
 } else if (digitalRead(SW_pin) == LOW && Joystick == 1) {
  Joystick = 0;
    if (Joystick == Joystick_old){
    }
    else {
    Serial.print("Universe OFF");
    Joystick_h = 1;
    }
 }
}

// --------------------------------------------------------------
// Modes of the system (Move Joystick)
// --------------------------------------------------------------
  if(Joystick == 0){
    Plante = 0;
  }
  else if(Joystick == 1){ 
  if(analogRead(X_pin)<300 || analogRead(Y_pin)<300){
    if(analogRead(X_pin)<analogRead(Y_pin)){
      Plante = 1; //Mercury
      Plante_Diameter = 4879; //km
      Plante_Weight = 0.33*pow(10,24); //kg
      Plante_Density = 5429; //kg/m^2
      Plante_Distance = 57.9*pow(10,6); //km
    }
    else if(analogRead(X_pin)>analogRead(Y_pin)){
      Plante = 2; //Venus
      Plante_Diameter = 12104; //km
      Plante_Weight = 4.87*pow(10,24); //kg
      Plante_Density = 5243; //kg/m^2
      Plante_Distance = 108.2*pow(10,6); //km

    }
  }
  else if(analogRead(X_pin)>700 || analogRead(Y_pin)>700){
    if (analogRead(X_pin)>analogRead(Y_pin)){
      Plante = 3; //Earth
      Plante_Diameter = 12756; //km
      Plante_Weight = 5.97*pow(10,24); //kg
      Plante_Density = 5514; //kg/m^2
      Plante_Distance = 149.6*pow(10,6); //km
    }
    else if(analogRead(X_pin)<analogRead(Y_pin)){
      Plante = 4; //Mars
      Plante_Diameter = 6792; //km
      Plante_Weight = 0.643*pow(10,24); //kg
      Plante_Density = 3934; //kg/m^2
      Plante_Distance = 228*pow(10,6); //km
    }
  }
  }
  Orbital_Velocity = sqrt(G*M_C/Plante_Distance); //km/s

// --------------------------------------------------------------
// Plantes printing
// -------------------------------------------------------------- 
  if(Plante == 0){
    if (Plante == Plante_old){
      if (Joystick == Joystick_old){
      }
      else {
      Serial.println();
      }
    }
  }
  else if(Plante == 1){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Mercury!");
    Serial.println();
    }
  }
  else if(Plante == 2){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Venus!");
    Serial.println();
    }
  }
  else if(Plante == 3){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Earth!");
    Serial.println();
    }
  }
  else if(Plante == 4){
    if (Plante == Plante_old){
    }
    else {
    Serial.print("Hello Mars!");
    Serial.println();
    }
  }
  Joystick_old = Joystick;
  Plante_old = Plante;

// --------------------------------------------------------------
// Servo Control
// -------------------------------------------------------------- 
  Real_angle = asin(Plante_Distance*Model_ratio_d/model_arm_length); //
  Modle_P_weight = 1000*3.1415926*(4/3)*pow((Plante_Diameter/2)*Model_ratio_r/1000,3); //kg
  weight_effect_speed = 10000*Modle_P_weight; //

  if(T_angle > Real_angle+angle_tolerance){
    S_speed = 75 - weight_effect_speed;
  }
  else if(T_angle < Real_angle-angle_tolerance){
    S_speed = 105 + weight_effect_speed;
  }
  else{
    S_speed = 90 + weight_effect_speed;
  }
    myservo.write(S_speed);

// --------------------------------------------------------------
// Motor Control
// -------------------------------------------------------------- 
  speed_ratio = Orbital_Velocity/50; //
  target = 250*speed_ratio+pos_old;

  // PID constants
  float kp = 1;
  float ki = 0.01;
  float kd = 0.025;
  
  long currT = micros(); // time difference
  float deltaT = ((float) (currT - prevT))/(pow(10,-6));
  prevT = currT;
  
  int e = pos - target; // error
  float dedt = (e-eprev)/(deltaT); // derivative
  eintegral = eintegral + e*deltaT; // integral
  float u = kp*e + kd*dedt + ki*eintegral; // control signal
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  
  setMotor(dir,pwr,PWM,IN1,IN2);
  eprev = e;
  pos_old = pos;  
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
