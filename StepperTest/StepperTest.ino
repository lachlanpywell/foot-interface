// Code sourced from: https://dronebotworkshop.com/big-stepper-motors/

/*
  Stepper Motor Test
  stepper-test01.ino
  Uses MA860H or similar Stepper Driver Unit
  Has speed control & reverse switch
  
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Defin pins

int reverseSwitch = 2;  // Push button for reverse
int driverPUL = 13;    // PUL- pin
int driverDIR = 3;    // DIR- pin
int spd = A0;     // Potentiometer

// Variables

int pd = 1000;       // Pulse Delay period
boolean setdir = LOW; // Set Direction

// Interrupt Handler

void revmotor (){

  setdir = !setdir;
  
}


void setup() {

  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
//  Serial.begin(9600);
  digitalWrite(driverDIR, HIGH);
  delayMicroseconds(500);
  digitalWrite(driverDIR, LOW);
  delayMicroseconds(500);
  digitalWrite(driverDIR, HIGH);
}

void loop() {
  
    //pd = map((analogRead(spd)),0,1023,2000,50);
//    Serial.print(pd);
//    Serial.print(" , ");
//    Serial.print(setdir);
//    Serial.println(" , Pulse High");
//    digitalWrite(driverDIR,LOW);
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);
 
}
