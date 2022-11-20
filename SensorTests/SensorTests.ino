/*
SENSOR TESTING
*/

void setup() {
  Serial.begin(57600);
}

void loop() {

  // -------------
  // FSRs
  // -------------
//  int FSR_0 = analogRead(A0);
//  int FSR_1 = analogRead(A1);
//  int FSR_2 = analogRead(A2);
//  int FSR_3 = analogRead(A3);
//  int FSR_4 = analogRead(A4);
//  int FSR_5 = analogRead(A5);
//
//  Serial.print("min:");
//  Serial.print(0);
//  Serial.print(",");
//  Serial.print("max:");
//  Serial.print(1023);
//  Serial.print(",");
//  Serial.print("FSR_0:");
//  Serial.print(FSR_0);
//  Serial.print(",");
//  Serial.print("FSR_1:");
//  Serial.print(FSR_1);
//  Serial.print(",");
//  Serial.print("FSR_2:");
//  Serial.print(FSR_2);
//  Serial.print(",");
//  Serial.print("FSR_3:");
//  Serial.print(FSR_3);
//  Serial.print(",");
//  Serial.print("FSR_4:");
//  Serial.print(FSR_4);
//  Serial.print(",");
//  Serial.print("FSR_5:");
//  Serial.println(FSR_5);

  // -------------
  // Roll/SC Hall Effect
  // -------------
  int HE_8 = analogRead(A8);
  int HE_9 = analogRead(A9);
  int HE_10 = analogRead(A10);

  Serial.print("min:");
  Serial.print(0);
  Serial.print(",");
  Serial.print("max:");
  Serial.print(1023);
  Serial.print(",");
  Serial.print("HE_8:");
  Serial.print(HE_8);
  Serial.print(",");
  Serial.print("HE_9:");
  Serial.print(HE_9);
  Serial.print(",");
  Serial.print("HE_10:");
  Serial.println(HE_10);

  delay(1);  // delay in between reads for stability
}
