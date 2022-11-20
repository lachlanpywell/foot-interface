#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>

// ROS ARRAY ELEMENTS
// 0: Rotation (Roll A, Roll B)
// 1: In/Out (Insertion, Withdrawal)
// 2: Pitch (Forward, Backward)
// 3: Yaw (Left, Right)
// 4: Spare
// Shared Control state is computed but not currently written to the topic

bool ROS_MODE = false;  // false = Serial, true = ROS
std_msgs::Float64MultiArray cmdMsgArrayFloat;
ros::Publisher FootInterface("interface_cmd", &cmdMsgArrayFloat);
ros::NodeHandle nh;

// CONSTANTS
const int ROLL_THRESHOLD_SLOW = 350;
const int ROLL_THRESHOLD_FAST = 480;
const int SHARED_CONTROL_ACTIVATE = 300;
const float ROTATION_SPEED_INCREMENT = 0.00005;
const float MIN_SPEED_ROTATION = 0.02;
const float MAX_SPEED_ROTATION = 0.04;

const int ROS_ROLL = 0;  // Array indices
const int ROS_INOUT = 1;
const int ROS_PITCH = 2;
const int ROS_YAW = 3;

// PIN DECLARATIONS
const int ROLL_A_PIN = A8;
const int ROLL_B_PIN = A9;
const int SHARED_CONTROL_PIN = A10;

// DECLARE AND INITIALISE GLOBAL VARIABLES
int rollAVal = 0;
int rollBVal = 0;
float cmdArrayFloat[5] = {
  0,
  0,
  0,
  0,
  0,
};
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
    if (cmdArrayFloat[ROS_ROLL] > -MAX_SPEED_ROTATION) {
      cmdArrayFloat[ROS_ROLL] -= ROTATION_SPEED_INCREMENT;
    }
  } else if (rollBVal > ROLL_THRESHOLD_SLOW) {
    cmdArrayFloat[ROS_ROLL] = -MIN_SPEED_ROTATION;
  } else {
    cmdArrayFloat[ROS_ROLL] = 0;
  }

  if (!ROS_MODE) {
    
    Serial.print(rollAVal);
    Serial.print(" , ");
    Serial.print(rollBVal);
    Serial.print(" , ");
    Serial.println(cmdArrayFloat[ROS_ROLL]);
  }
}
