#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
#include <ACE128.h>
#include <ACE128map12345678.h>

// ROS ARRAY ELEMENTS
// 0: Rotation (Roll A, Roll B)
// 1: In/Out (Insertion, Withdrawal)
// 2: Pitch (Forward, Backward)
// 3: Yaw (Left, Right)
// 4: Spare (always zero)
// Shared Control state is computed but not currently written to the topic

bool ROS_MODE = true;  // false = Serial (used for debugging), true = ROS

// ROS Publisher
std_msgs::Float64MultiArray cmdMsgArrayFloat;
ros::Publisher FootInterface("interface_cmd", &cmdMsgArrayFloat);
ros::NodeHandle nh;

// Create Global Objects
ACE128 myACE(36, 38, 40, 42, 28, 30, 32, 34, (uint8_t*)encoderMap_12345678); // encoder

// CONSTANTS
// Hall Effect Sensors
const int ROLL_THRESHOLD_SLOW = 350;
const int ROLL_THRESHOLD_FAST = 480;
const int SHARED_CONTROL_ACTIVATE = 300; // threshold read from hall effect sensor to toggle shared control mode on/off
const float ROTATION_SPEED_INCREMENT = 0.00002; // when transitioning from slow to fast the speed increments by this amount each loop until the faster speed is reached
const float MIN_SPEED_ROTATION = 0.02;
const float MAX_SPEED_ROTATION = 0.04;

// Pressure Sensors
const int FSR_SLOW_THRESHOLD = 100;
const int FSR_FAST_THRESHOLD = 25;
const int MIN_TAP_TIME = 50;
const int MAX_TAP_TIME = 1700;
const float MIN_SPEED_INOUT = 0.002;
const float MAX_SPEED_INOUT = 0.007;
const float INOUT_SPEED_INCREMENT = 0.00001;

// Encoders
// LR = Left/Right encoder
// FB = Forward/Back encoder
const int ENCODER_LR_FULL_SCALE = 1023;
const int ENCODER_FB_FULL_SCALE = 128;
const int ENCODER_LR_RANGE = 800; // i.e. 0-90 is the max revolution possible rotation of the gear
const int ENCODER_FB_RANGE = 90;

const int ENCODER_LR_HOME = 500; // TO BE TESTED
const int ENCODER_LR_WINDOW = 25;// TO BE TESTED
const int ENCODER_FB_HOME = 60;// TO BE TESTED
const int ENCODER_FB_WINDOW = 4;// TO BE TESTED

const int ENCODER_LR_UPPER_SLOW = 700;
const int ENCODER_LR_UPPER_FAST = 900;
const int ENCODER_LR_LOWER_SLOW = 300;
const int ENCODER_LR_LOWER_FAST = 100;

const int ENCODER_FB_UPPER_SLOW = 110;
const int ENCODER_FB_UPPER_FAST = 80;
const int ENCODER_FB_LOWER_SLOW = 40;
const int ENCODER_FB_LOWER_FAST = 10;

const int MAX_SPEED_PITCH = 0.05;
const int MIN_SPEED_PITCH = 0.02;
const int PITCH_SPEED_INCREMENT = 0.00002;

const int MAX_SPEED_YAW = 0.06;
const int MIN_SPEED_YAW = 0.02;
const int YAW_SPEED_INCREMENT = 0.00002;

// Array indices
// These can be changed to map foot movements to different camera movements
const int ROS_ROLL = 0;
const int ROS_INOUT = 1;
const int ROS_PITCH = 2;
const int ROS_YAW = 3;

// PIN DECLARATIONS
const int ROLL_A_PIN = A8;
const int ROLL_B_PIN = A9;
const int SHARED_CONTROL_PIN = A10;
const int FSR_0_PIN = A0;
const int FSR_1_PIN = A1;
const int FSR_2_PIN = A2;
const int FSR_3_PIN = A3;
const int FSR_4_PIN = A4;
const int FSR_5_PIN = A5;
const int LIMIT_SWITCH_1_PIN = 14;
const int LIMIT_SWITCH_2_PIN = 15;
const int LIMIT_SWITCH_3_PIN = 16;
const int LIMIT_SWITCH_4_PIN = 17;
const int CS_PIN = 5;
const int CLOCK_PIN = 6;
const int DATA_PIN = 7;

// DECLARE AND INITIALISE GLOBAL VARIABLES
// Roll
int rollAVal = 0;
int rollBVal = 0;
int sharedControlVal = 0;
float cmdArrayFloat[5] = {
  0,
  0,
  0,
  0,
  0,
};
bool sharedControlMode = false;  // false = shared control OFF
bool sharedControlCurrState = false;
bool sharedControlPrevState = false;
long currTimeROS = 0;
long prevTimeROS = 0;
char tempROSLogging[20];

// Insertion / Withdrawal
int insertionPressure = 0;
int withdrawalPressure = 0;
int temp1 = 0;
int temp2 = 0;
int temp3 = 0;
float insertionSpeed = 0.0;
float withdrawalSpeed = 0.0;
bool prevStateIns = false;
bool prevStateWith = false;
bool currStateIns = false;
bool currStateWith = false;
bool insertionStopped = false;
bool withdrawalStopped = false;
bool footLifted = false;
int countTapIns = 0;
int countTapWith = 0;
long timeAIns = 0;
long timeAWith = 0;
long timeLastIns = 0;
long timeLastWith = 0;
long startTimeIns = 0;
long startTimeWith = 0;

// Encoders
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


// ------------------- //
// SETUP
// ------------------- //
void setup() {

  // Start ROS Node
  if (ROS_MODE) {
    nh.initNode();
    nh.advertise(FootInterface);
  } else {
    Serial.begin(57600);
  }

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

// ------------------- //
// LOOP
// ------------------- //
void loop() {

  // ------------------- //
  // ROLL
  // ------------------- //
  // If both sensors are pressed, Roll A takes precedence
  // Read Roll Hall Effect Sensor Raw Values
  rollAVal = analogRead(ROLL_A_PIN);
  rollBVal = analogRead(ROLL_B_PIN);

  // Roll A
  if (rollAVal > ROLL_THRESHOLD_FAST) {
    // ramp to max speed
    if (cmdArrayFloat[ROS_ROLL] < MAX_SPEED_ROTATION) {
      cmdArrayFloat[ROS_ROLL] += ROTATION_SPEED_INCREMENT;
    }
  } else if (rollAVal > ROLL_THRESHOLD_SLOW) {
    cmdArrayFloat[ROS_ROLL] = MIN_SPEED_ROTATION;
  }

  // Roll B
  else if (rollBVal > ROLL_THRESHOLD_FAST) {
    // ramp to max speed
    if (cmdArrayFloat[ROS_ROLL] > -MAX_SPEED_ROTATION) {
      cmdArrayFloat[ROS_ROLL] -= ROTATION_SPEED_INCREMENT;
    }
  } else if (rollBVal > ROLL_THRESHOLD_SLOW) {
    cmdArrayFloat[ROS_ROLL] = -MIN_SPEED_ROTATION;
  }  else {
    // if both sensors not pressed
    cmdArrayFloat[ROS_ROLL] = 0;
  }


  // ------------------- //
  // INSERTION
  // ------------------- //
  // Read Sensor Values
  temp1 = analogRead(FSR_0_PIN);
  temp2 = analogRead(FSR_1_PIN);
  temp3 = analogRead(FSR_2_PIN);
  insertionPressure = (temp1 + temp2 + temp3) / 3; // average pressure sensor readings

  temp1 = analogRead(FSR_3_PIN);
  temp2 = analogRead(FSR_4_PIN);
  temp3 = analogRead(FSR_5_PIN);
  withdrawalPressure = (temp1 + temp2 + temp3) / 3; // average pressure sensor readings

  pressureStates(insertionPressure, currStateIns); // pressure sensors either ON/OFF based on current reading (acting like a switch)
  pressureStates(withdrawalPressure, currStateWith);

  // Detect if the foot is lifted off the pedal
  if (withdrawalPressure < FSR_SLOW_THRESHOLD && insertionPressure < FSR_SLOW_THRESHOLD) {
    footLifted = true;
  } else {
    footLifted = false;
  }

  // Insertion
  doubleTapCount(currStateIns, currStateWith, prevStateIns, prevStateWith, countTapIns, startTimeIns, timeLastIns, timeAIns);
  insertionSpeed = doubleTapSpeed(countTapIns, insertionPressure, withdrawalPressure, 1);

  // Withdrawal
  doubleTapCount(currStateWith, currStateIns, prevStateWith, prevStateIns, countTapWith, startTimeWith, timeLastWith, timeAWith);
  withdrawalSpeed = doubleTapSpeed(countTapWith, withdrawalPressure, insertionPressure, -1);

  cmdArrayFloat[ROS_INOUT] = insertionSpeed > abs(withdrawalSpeed) ? insertionSpeed : withdrawalSpeed;

  prevStateIns = currStateIns;
  prevStateWith = currStateWith;


  // ------------------- //
  // PITCH
  // ------------------- //
  if (encoderCalibrationMode) {

    encoderLimitSwitchFB = 22; // TO DO

  }
  else {
    limitSwitch1State = digitalRead(LIMIT_SWITCH_1_PIN);
    limitSwitch2State = digitalRead(LIMIT_SWITCH_2_PIN);
    encoderValFB = myACE.rawPos();
    encoderValMappedFB = encoderMapping(encoderValFB, limitSwitch1State, limitSwitch2State, encoderLimitSwitchFB , ENCODER_FB_FULL_SCALE, ENCODER_FB_RANGE);

    if (encoderValMappedFB < ENCODER_FB_LOWER_FAST) {
      // ramp to max speed
      if (cmdArrayFloat[ROS_PITCH] < -MAX_SPEED_PITCH) {
        cmdArrayFloat[ROS_PITCH] -= PITCH_SPEED_INCREMENT;
      }
    } else if (encoderValMappedFB < ENCODER_FB_LOWER_SLOW || (cmdArrayFloat[ROS_PITCH] = -MIN_SPEED_PITCH && encoderValMappedFB < (ENCODER_FB_HOME - ENCODER_FB_WINDOW))) {
      // slow speed
      cmdArrayFloat[ROS_PITCH] = -MIN_SPEED_PITCH;
    }
    else if (encoderValMappedFB > ENCODER_FB_UPPER_FAST) {
      if (cmdArrayFloat[ROS_PITCH] < MAX_SPEED_PITCH) {
        cmdArrayFloat[ROS_PITCH] += PITCH_SPEED_INCREMENT;
      }
    }
    else if (encoderValMappedFB > ENCODER_FB_UPPER_SLOW || (cmdArrayFloat[ROS_PITCH] = MIN_SPEED_PITCH && encoderValMappedFB > (ENCODER_FB_HOME + ENCODER_FB_WINDOW))) {
      // slow speed
      cmdArrayFloat[ROS_PITCH] = MIN_SPEED_PITCH;
    }
    else {
      cmdArrayFloat[ROS_PITCH] = 0.0;
    }

  }


  // ------------------- //
  // YAW
  // ------------------- //
  if (encoderCalibrationMode) {

    encoderLimitSwitchLR = 22; // TO DO

  }
  else {
    limitSwitch3State = digitalRead(LIMIT_SWITCH_3_PIN);
    limitSwitch4State = digitalRead(LIMIT_SWITCH_4_PIN);
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
    if (encoderValMappedLR < ENCODER_LR_LOWER_FAST) {
      // ramp to max speed
      if (cmdArrayFloat[ROS_YAW] < -MAX_SPEED_YAW) {
        cmdArrayFloat[ROS_YAW] -= YAW_SPEED_INCREMENT;
      }
    } else if (encoderValMappedLR < ENCODER_LR_LOWER_SLOW || (cmdArrayFloat[ROS_YAW] = -MIN_SPEED_YAW && encoderValMappedLR < (ENCODER_LR_HOME - ENCODER_LR_WINDOW))) {
      // slow speed
      cmdArrayFloat[ROS_YAW] = -MIN_SPEED_YAW;
    }
    else if (encoderValMappedLR > ENCODER_LR_UPPER_FAST) {
      if (cmdArrayFloat[ROS_YAW] < MAX_SPEED_YAW) {
        cmdArrayFloat[ROS_YAW] += YAW_SPEED_INCREMENT;
      }
    }
    else if (encoderValMappedLR > ENCODER_LR_UPPER_SLOW || (cmdArrayFloat[ROS_YAW] = MIN_SPEED_YAW && encoderValMappedLR > (ENCODER_LR_HOME + ENCODER_LR_WINDOW))) {
      // slow speed
      cmdArrayFloat[ROS_YAW] = MIN_SPEED_YAW;
    }
    else {
      cmdArrayFloat[ROS_YAW] = 0.0;
    }
  }

  itoa(encoderValMappedLR, tempROSLogging, 10);
  nh.loginfo(tempROSLogging);

  itoa(encoderValMappedFB, tempROSLogging, 10);
  nh.loginfo(tempROSLogging);

  // ------------------- //
  // SHARED CONTROL
  // ------------------- //
  // Read shared control hall effect sensor
  sharedControlVal = analogRead(SHARED_CONTROL_PIN);

  if (sharedControlVal > SHARED_CONTROL_ACTIVATE) {
    sharedControlCurrState = true;
  } else {
    sharedControlCurrState = false;
  }

  if (sharedControlCurrState && !sharedControlPrevState) {
    sharedControlMode = !sharedControlMode;  // not currently written to ROS topic
  }

  sharedControlPrevState = sharedControlCurrState;

  // ------------------- //
  // PUBLISH ROS TOPIC
  // ------------------- //
  currTimeROS = millis();
  if (currTimeROS - prevTimeROS > 100) {

    if (ROS_MODE) {
      cmdMsgArrayFloat.data = cmdArrayFloat;
      cmdMsgArrayFloat.data_length = 5;
      FootInterface.publish(&cmdMsgArrayFloat);
      nh.spinOnce();
    }
    prevTimeROS = currTimeROS;
  }

  delay(1);  // for analog read stability
}


// ------------------- //
// FUNCTIONS
// ------------------- //
// Determine if the force sensing resistor is in the ON or OFF state based on current pressure reading and the pressure threshold/setpoint
void pressureStates(int pressureVal, bool &currState) {
  // Check values against threshold
  if (pressureVal < FSR_SLOW_THRESHOLD) {
    currState = false;
  } else {
    currState = true;
  }
}

// Count the number of state changes occuring in a given time period (i.e. count the number of taps performed on each group of sensors)
void doubleTapCount(bool currStateA, bool currStateB, bool prevStateA, bool prevStateB, int &countTap, long &startTime, long &timeLast, long &timeA) {

  // Record State Change Times (when sensor detects foot lift or tap on insertion/withdrawal sensors)
  if ((currStateA != prevStateA) && currStateB) {
    if (countTap == 0 && !currStateA) {
      startTime = millis();
      countTap += 1;
      timeLast = startTime;
    } else if ((footLifted) && currStateA) {
      countTap = 0;
      //      latchCommand = false;
    } else if (countTap > 0) {
      timeA = millis();

      if (abs(timeA - timeLast) > MIN_TAP_TIME) {
        countTap += 1;
      }
      if (abs(timeA - startTime) > MAX_TAP_TIME) {
        countTap = 1;
        startTime = timeA;
      }
      timeLast = timeA;
    }

  }
}

// Calculate if the insertion/withdrawal command is the low, high or off speed based on the current pressure reading
// and number of taps performed
float doubleTapSpeed(int &countTap, int pressureA, int pressureB, int dir) {
  float output = 0.0;
  if (countTap == 4) { // i.e. OFF->ON->OFF->ON = 4
    if (pressureA > FSR_SLOW_THRESHOLD) {
      if (pressureB < FSR_FAST_THRESHOLD) {
        if (cmdArrayFloat[ROS_INOUT]*dir < MAX_SPEED_INOUT) {
          output = cmdArrayFloat[ROS_INOUT] + INOUT_SPEED_INCREMENT * dir;
        }
        else {
          output = cmdArrayFloat[ROS_INOUT];
        }
      } else {
        output = MIN_SPEED_INOUT * dir;
      }
    }
  }

  return output;
}

// Map the encoder raw/absolute value to 0 - Configured Full Max (not necessarilly full scale) (i.e. all the way to one end will read zero, other end read configured max)
// This ensures if the encoders are removed from the assembly when placed in have rotated to a new location (absolute encoders) the setpoints used for the home position
// and slow/fast spots do not need to be updated, rather they can be 'recalibrated' in code using the limit switches
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

// DEBUGGING (write ROS log message)
// itoa(rollAVal, tempROSLogging, 10);
// nh.loginfo(tempROSLogging);
