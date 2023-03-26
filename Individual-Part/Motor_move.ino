#include <util/atomic.h> 

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM 7
#define IN1 6
#define IN2 5

volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int pos = 0; 
int target = 0;
int pos_old = 0;

double Orbital_Velocity = 0; //km/s
int speed_ratio = 0; //

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  }

void loop() {
  speed_ratio = Orbital_Velocity/50; //
  target = 255*speed_ratio+pos;

  // PID constants
  float kp = 1;
  float ki = 0.01;
  float kd = 0.025;
  
  long currT = micros(); // time difference
  float deltaT = ((float) (currT - prevT))/(pow(10,-6));
  prevT = currT;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  int e = pos - target; // error
  float dedt = (e-eprev)/(deltaT); // derivative
  eintegral = eintegral + e*deltaT; // integral
  float u = kp*e + kd*dedt + ki*eintegral; // control signal
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  if( pwr <0 ){
    pwr = 0;
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
