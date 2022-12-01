#include <ACE128.h>
#include <ACE128map12345678.h>

ACE128 myACE(36, 38, 40, 42, 28, 30, 32, 34, (uint8_t*)encoderMap_12345678);

// PIN DECLARATIONS
const int LIMIT_SWITCH_1_PIN = 14;
const int LIMIT_SWITCH_2_PIN = 15;
const int LIMIT_SWITCH_3_PIN = 16;
const int LIMIT_SWITCH_4_PIN = 17;
const int CS_PIN = 5;
const int CLOCK_PIN = 6;
const int DATA_PIN = 7;


const int ENCODER_LR_FULL_SCALE = 1023;
const int ENCODER_FB_FULL_SCALE = 128;
const int ENCODER_LR_RANGE = 800; // i.e. 0-90 is the max revolution possible rotation of the gear
const int ENCODER_FB_RANGE = 90;


// GLOBALS
int limitSwitch1State = true;
int limitSwitch2State = true;
int limitSwitch3State = true;
int limitSwitch4State = true;
int encoderValFB = 0; // forward back 0-127
int encoderValLR = 0; // left right 0-1023
int encoderValMappedLR = 0;
int encoderValMappedFB = 0;
int encoderValPrevLR = 0;
int encoderValPrevFB = 0;
bool encoderCalibrationMode = false;
int encoderLimitSwitchLR = 0;
int encoderLimitSwitchFB = 0;

void setup() {
  Serial.begin(57600);
  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP );
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP );
  pinMode(LIMIT_SWITCH_3_PIN, INPUT_PULLUP );
  pinMode(LIMIT_SWITCH_4_PIN, INPUT_PULLUP );
  myACE.begin();
  pinMode(CS_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
}

void loop() {

  if (encoderCalibrationMode) {
    encoderLimitSwitchLR = 7; // drive motor till limit switch pressed and record value
    encoderLimitSwitchFB = 22;

  }
  else {
    limitSwitch1State = digitalRead(LIMIT_SWITCH_1_PIN);
    limitSwitch2State = digitalRead(LIMIT_SWITCH_2_PIN);
    limitSwitch3State = digitalRead(LIMIT_SWITCH_3_PIN);
    limitSwitch4State = digitalRead(LIMIT_SWITCH_4_PIN);

    Serial.print(limitSwitch1State);
    Serial.print(" , ");
    Serial.print(limitSwitch2State);
    Serial.print(" , ");
    Serial.print(limitSwitch3State);
    Serial.print(" , ");
    Serial.print(limitSwitch4State);

    encoderValFB = myACE.rawPos();
    encoderValMappedFB = encoderMapping(encoderValFB, limitSwitch1State, limitSwitch2State,encoderLimitSwitchFB , ENCODER_FB_FULL_SCALE, ENCODER_FB_RANGE);

    digitalWrite(CS_PIN, HIGH);
    digitalWrite(CS_PIN, LOW);
    encoderValLR = 0;

    for (int i = 0; i < 16; i++) {
      digitalWrite(CLOCK_PIN, LOW);
      digitalWrite(CLOCK_PIN, HIGH);
      byte b = digitalRead(DATA_PIN) == HIGH ? 1 : 0;
      encoderValLR += b * pow(2, 10 - (i + 1));
    }

    // Status bits, not written or read
    for (int i = 0; i < 6; i++) {
      digitalWrite(CLOCK_PIN, LOW);
      digitalWrite(CLOCK_PIN, HIGH);
    }

    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(CLOCK_PIN, HIGH);

    encoderValMappedLR = encoderMapping(encoderValLR, limitSwitch3State, limitSwitch4State, encoderLimitSwitchLR, ENCODER_LR_FULL_SCALE, ENCODER_LR_RANGE);

    Serial.print(" , ");
    Serial.print(encoderValLR);
    Serial.print(" , ");
    Serial.print(encoderValMappedLR);
    
    Serial.print(" , ");
    Serial.print(encoderValFB);
    Serial.print(" , ");
    Serial.println(encoderValMappedFB);

    delay(1);        // delay in between reads for stability

  }
}

int encoderMapping(int rawVal, bool limitSwitch1, bool limitSwitch2, int limitSwitchZero, int ENCODER_FULL_SCALE, int FULL_RANGE) {
  int output = 0;

  if (rawVal < limitSwitchZero) {
    output = ENCODER_FULL_SCALE - limitSwitchZero + rawVal;
  } else {
    output = rawVal - limitSwitchZero;
  }

  // Clamp encoder val to min/max if limit switch active
  if (!limitSwitch1) {
    output = 0;
  } else if (!limitSwitch2) {
    output = FULL_RANGE;
  }

  return output;
}
