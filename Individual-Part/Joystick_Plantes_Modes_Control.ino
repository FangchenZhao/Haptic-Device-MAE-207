// Arduino pin numbers
#define SW_pin 13 // digital pin connected to switch output orange
#define X_pin A4 // analog pin connected to X output green
#define Y_pin A5 // analog pin connected to Y output blue

double Joystick_h = 0; //Joystick Case hold
int Joystick = 0; //Joystick Case LOW (pressed)
double Plante = 0;
double Joystick_old = 0;

double G = 6.67428*pow(10,-20); //km^3 kg^-1 s^-2
double M_C = 1.98892*pow(10,30); //kg (mass of center [sun])

int Plante_Diameter = 0; //km
int Plante_Weight = 0; //kg
int Plante_Density = 0; //kg/m^2
int Plante_Distance = 0; //km
int Orbital_Period = 0; //days
int Orbital_Velocity = 0; //km/s

void setup() {
  pinMode(SW_pin, INPUT);
  pinMode(X_pin, INPUT); // set Joystick pin to be an input
  pinMode(Y_pin, INPUT); // set Joystick pin to be an input
  Serial.begin(115200);
}

void loop() {
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

}
