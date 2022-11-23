#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>

// ROS ARRAY ELEMENTS
// 0: Rotation (Roll A, Roll B)
// 1: In/Out (Insertion, Withdrawal)
// 2: Pitch (Forward, Backward)
// 3: Yaw (Left, Right)
// 4: Spare (always zero)
// Shared Control state is computed but not currently written to the topic

bool ROS_MODE = true;  // false = Serial, true = ROS
std_msgs::Float64MultiArray cmdMsgArrayFloat;
ros::Publisher FootInterface("interface_cmd", &cmdMsgArrayFloat);
ros::NodeHandle nh;

// CONSTANTS
// Hall Effect Sensors
const int ROLL_THRESHOLD_SLOW = 350;
const int ROLL_THRESHOLD_FAST = 480;
const int SHARED_CONTROL_ACTIVATE = 300;
const float ROTATION_SPEED_INCREMENT = 0.00002;
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

// Array indices
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

// DECLARE AND INITIALISE GLOBAL VARIABLES
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

  pressureStates(insertionPressure, currStateIns);
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
  Serial.print(countTapWith);
  Serial.print(" , ");
  Serial.println(cmdArrayFloat[ROS_INOUT], 4);

  prevStateIns = currStateIns;
  prevStateWith = currStateWith;


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
// INSERTION/WITHDRAWAL
// ------------------- //
void pressureStates(int pressureVal, bool &currState) {
  // Check values against threshold
  if (pressureVal < FSR_SLOW_THRESHOLD) {
    currState = false;
  } else {
    currState = true;
  }
}

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

// Calculate if the insertion/withdrawal command is the low, high or off speed.
float doubleTapSpeed(int &countTap, int pressureA, int pressureB, int dir) {
  float output = 0.0;
  if (countTap == 4) {
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
//  if (cmdArrayFloat[ROS_ROLL] > -MAX_SPEED_ROTATION) {
//      cmdArrayFloat[ROS_ROLL] -= ROTATION_SPEED_INCREMENT;
//    }
    
  }

  return output;
}

// DEBUGGING (write ROS log message)
// itoa(rollAVal, tempROSLogging, 10);
// nh.loginfo(tempROSLogging);
