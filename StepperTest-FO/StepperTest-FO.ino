// Define pins

int driverPUL = 7;    // PUL- pin
int driverDIR = 9;    // DIR- pin

// Variables

int pd = 100000000000;       // Pulse Delay period
void setup() {
  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
  digitalWrite(driverDIR,HIGH);
  delayMicroseconds(200);
}

void loop() {
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(200);
  //  delay(10);
    digitalWrite(driverPUL,LOW);
//delay(10);
    delayMicroseconds(200);
 
}
