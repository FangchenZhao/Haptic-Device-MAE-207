#include <Servo.h>
Servo myservo;

// Define the servo pin:
#define servoPin A0

int S_speed = 0; //0-180 (0:CCW max speed, 90:Stop, 180:CW max speed)
double angle_tolerance = 20; //angle tolerance (so large because of the wobble servo)
double weight_effect_speed = 0; //

int Real_angle = 0; //
int Real_Distance = 0; //mm

int Modle_P_weight = 0; //kg
double model_arm_length = 12*25.4; //mm
double Model_ratio_d = 1.3368*pow(10,-6); //mm/km (model to real object [orbital distance])
double Model_ratio_r = 5.20598*pow(10,-3); //mm/km (model to real object [radius])
double material_density = 1000; //kg/m^3 (60% PLA)
double gear_ratio = 25; //Gearbox and belt gear

// Input Data
double T_angle = 0; //angle of top in body x-direction
double Plante_Diameter = 0; //km
double Plante_Distance = 0; //Distance to the center (planet distance from sun)

void setup() {
  myservo.attach(servoPin);
}

void loop() {
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
}
