// roscore
// rosrun rosserial_python serial_node.py /dev/ttyACM1
// need to stop rosserial node before uploading code from IDE!
// rostopic echo /interface_cmd

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
#include <Servo.h>
#include <Ewma.h>

Servo servoA; // A = Left/Right
Servo servoB; // B = Up/Down
Ewma CombinedValAFilter(0.1);
Ewma ControlValAFilter(0.1);
Ewma CombinedValBFilter(0.1);
Ewma ControlValBFilter(0.1);

// ROS ARRAY ELEMENTS
// 0: Roll A
// 1: Roll B
// 2: Insertion
// 3: Withdrawal
// 4: Forward (Pitch)
// 5: Back (Pitch)
// 6: Left (Yaw)
// 7: Right (Yaw)
// 8: Shared Control

// X, Y, Pitch, Rotation
// X (6,7)
// Y (4,5)
// Pitch (2,3)
// Rotation (0,1)

// VALUES:
//0: Off
//1: Slow
//2: Ramp Up
//3: Fast
//4: Ramp Down (Not used)

bool ROS_MODE = true; // false = Serial, true = ROS

std_msgs::Float64MultiArray cmdMsgArrayFloat;
ros::Publisher FootInterface("interface_cmd", &cmdMsgArrayFloat);
ros::NodeHandle nh;

// USER PREFERENCE
bool WORKSPACE_RANGE = true;     // true = full workspace range, false = smaller range
bool DECOUPLED_CONTROL = false;  // true = coupled, false = decoupled (XY translational movements only)
bool PIDA_MODE = true; // false = snap to home, true = compliance mode
bool PIDB_MODE = true; // false = snap to home, true = compliance mode

// CONSTANTS
const int ROLL_THRESHOLD_SLOW = 350;
const int ROLL_THRESHOLD_FAST = 480;
const int SHARED_CONTROL_ACTIVATE = 300;
const long int RAMP_COUNT_THRESHOLD = 300;
const long int RAMP_THRESHOLD_INS = 300;
const long int PITCH_RAMP_COUNT_THRESHOLD = 300;
const int FSR_SLOW_THRESHOLD = 200;
const int FSR_FAST_THRESHOLD = 25;
const int MIN_TAP_TIME = 50;
const int MAX_TAP_TIME = 2000;
const long LED_BLINK_SLOW = 1000;
const long LED_BLINK_FAST = 200;
const int MEMBRANE_POT_UPPER_LIMIT = 645;
const int MEMBRANE_POT_LOWER_LIMIT = 550;
const float MIN_SPEED_ROTATION = 0.02;
const float MAX_SPEED_ROTATION = 0.04;
const float MIN_SPEED_Y = 0.02;
const float MAX_SPEED_Y = 0.05;
const float MIN_SPEED_X = 0.02;
const float MAX_SPEED_X = 0.06;
int PITCH_FORWARD_FAST = 0;  // not declared as const as calculated in setup based on WORKSPACE_RANGE variable
int PITCH_FORWARD_SLOW = 0;
int PITCH_BACKWARD_SLOW = 0;
int PITCH_BACKWARD_FAST = 0;
int YAW_LEFT_FAST = 0;
int YAW_LEFT_SLOW = 0;
int YAW_RIGHT_SLOW = 0;
int YAW_RIGHT_FAST = 0;
const int HOME_MEMBRANE_LOWER = 350;
const int HOME_MEMBRANE_UPPER = 400;
const int VIBRATION_DURATION = 400;
const long int YAW_RAMP_COUNT_THRESHOLD = 400;
const int ENCODER_HOME_LOWER = 550;
const int ENCODER_HOME_UPPER = 600;
const int SERVO_RESTING_A = 1500;
const int SERVO_RESTING_B = 1478;

// PIN DECLARATIONS
int insertionPin = A2;   // defaults if no calibration performed
int withdrawalPin = A4;  // defaults if no calibration performed
const int calSwitchPin = 2;
const int calLEDPin = 3;
const int EStopPin = 8;
const int membranePotPin = A15;
const int vibrationMotor1Pin = 9;
const int vibrationMotor2Pin = 10;
const int PIN_CS = 5;
const int PIN_CLOCK = 6;
const int PIN_DATA = 7;
const int ServoAPin = 11;
const int ServoBPin = 4;
const int servoAHall1Pin = A7;
const int servoAHall2Pin = A11;
const int servoBHall1Pin = A13;
const int servoBHall2Pin = A14;

// VARIABLES
int servoAHall1 = 0;
int servoAHall2 = 0;
int servoBHall1 = 0;
int servoBHall2 = 0;
int combinedValA = 0;
int combinedValPrevA = 0;
int combinedValB = 0;
int combinedValPrevB = 0;
int maxSpeedValA = 0; // configured based on PID mode
int maxSpeedValB = 0; // ....
int minSpeedValA = 0; // ....
int minSpeedValB = 0; // ....
int PIDTargetA = 0; // ....
int PIDTargetB = 0; // ....
int combinedValAFiltered = 0;
int controlValAFiltered = 0;
int combinedValBFiltered = 0;
int controlValBFiltered = 0;
float kpA = 0; // ....
float kdA = 0;// ....
float kiA = 0;// ....
float kpB = 0;// ....
float kdB = 0;// ....
float kiB = 0;// ....
long currTA = 0;// ....
float deltaTA = 0;// ....
int errorA = 0;// ....
float dedtA = 0;// ....
float uA = 0;// ....
float uPrevA = 0;// ....
long prevTA = 0;
float ePrevA = 0;
float eIntegralA = 0;
int controlSignalA = 0;
long currTB = 0;// ....
float deltaTB = 0;// ....
int errorB = 0;// ....
float dedtB = 0;// ....
float uB = 0;// ....
float uPrevB = 0;// ....
long prevTB = 0;// ....
float ePrevB = 0;// ....
float eIntegralB = 0;// ....

int thresholdLowA = 440; // to be tested
int thresholdHighA = 525;
int thresholdLowB = 375;
int thresholdHighB = 475; // to be tested
int homePosA = (thresholdLowA + thresholdHighA)/2;
int homePosB = (thresholdLowB + thresholdHighB)/2;
int controlSignalB = 0;// configured based on PID mode
int rollAVal = 0;
int rollBVal = 0;
int sharedControlVal = 0;
bool sharedControlActive = false;
int membranePotVal = 0;
int membranePotValPrev = 0;
int membraneAvg = 0;
int cmdArray[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float cmdArrayFloat[5] = {
  0,
  0,
  0,
  0,
  0,
};
long int rollARampCounter = 0;
long int rollBRampCounter = 0;
long int insertionRampCounter = 0;
long int withdrawalRampCounter = 0;
char tempROSLogging[20];
long currTimeROS = 0;
long prevTimeROS = 0;
long prevTimeLEDSlow = 0;
long prevTimeLEDFast = 0;
long prevControlStateTime = 0;
long prevTimeVibMem = 0;
int insertionPressure = 0;
int withdrawalPressure = 0;
bool prevStateIns = false;
bool prevStateWith = false;
bool currStateIns = false;
bool currStateWith = false;
bool timeInterlock = false;
bool timeInterlock2 = false;
bool insertionStopped = false;
bool interlocked = false;
int countTapIns = 0;
long countTapWith = 0;
long timeAIns = 0;
long timeAWith = 0;
long timeLastIns = 0;
long timeLastWith = 0;
long startTimeIns = 0;
long startTimeWith = 0;
bool footLifted = false;
bool withdrawalStopped = false;
bool calSwitch = false;
bool vibrationInhibitIns = false;
int sensorNumInsertion = 100;
int sensorNumWithdrawal = 100;
int tempSort = 0;
int tempPin = 0;
int maxVal[6] = { 0, 0, 0, 0, 0, 0 };
int A0val = 0;
int A1val = 0;
int A2val = 0;
int A3val = 0;
int A4val = 0;
int A5val = 0;
bool firstLoopCal = true;
int LEDState = LOW;
bool EStopCurrState = LOW;
bool EStopPrevState = HIGH;
bool EStopActive = false;
int pitchRampCounter = 0;
bool vibMotor1Active = false;
bool vibrationInhibitMembrane = false;
bool vibMotor2Active = false;
bool vibrationInhibitEncoder = false;
int encoderValMapped = 0;
bool clampLeft = false;
bool clampRight = false;
long prevTimeVibEnc = 0;
int yawRampCounter = 0;

// ------------------- //
// SETUP
// ------------------- //
void setup() {

  // Configure XY translational movement setpoints for low/high speed
  if (WORKSPACE_RANGE) {
    // Full Range Setpoints
    //90 - 710

    PITCH_FORWARD_FAST = 550;   // 625 for membrane pot, 650 for encoder (nominal starting points)
    PITCH_FORWARD_SLOW = 450;   // 605
    PITCH_BACKWARD_SLOW = 300;  // 590
    PITCH_BACKWARD_FAST = 200;  // 570
    // Desired: 150 - 600
    // Avg: 375
    // Home: 350 - 400

    YAW_LEFT_FAST = 800;
    YAW_LEFT_SLOW = 700;
    YAW_RIGHT_SLOW = 450;
    YAW_RIGHT_FAST = 350;

    // Encoder home position: 370, 430
    // Desired Range: 280 - 870
    // Avg: 575
    // Home: 550 - 600

  } else {
    // Small Range Setpoints
    PITCH_FORWARD_FAST = 500;   // 615
    PITCH_FORWARD_SLOW = 450;   // 605
    PITCH_BACKWARD_SLOW = 300;  // 590
    PITCH_BACKWARD_FAST = 250;  //580
    YAW_LEFT_FAST = 750;
    YAW_LEFT_SLOW = 650;
    YAW_RIGHT_SLOW = 500;
    YAW_RIGHT_FAST = 400;
  }



  servoA.attach(ServoAPin);
  servoA.writeMicroseconds(SERVO_RESTING_A);
//  servoB.attach(ServoBPin);
//  servoB.writeMicroseconds(SERVO_RESTING_B);

  // Arduino Pin Configuration
  pinMode(calLEDPin, OUTPUT);
  pinMode(calSwitchPin, INPUT_PULLUP);
  pinMode(EStopPin, INPUT_PULLUP);
  pinMode(vibrationMotor1Pin, OUTPUT);
  pinMode(vibrationMotor2Pin, OUTPUT);
  digitalWrite(vibrationMotor1Pin, LOW);
  digitalWrite(vibrationMotor2Pin, LOW);
  digitalWrite(calLEDPin, LOW);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);

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
  // E-Stop//
  // ------------------- //
  EStopCurrState = digitalRead(EStopPin);

  // if (EStopCurrState == LOW && EStopPrevState == HIGH) {
  if (EStopCurrState == HIGH && EStopPrevState == LOW) {
    EStopActive = !EStopActive;
  }
  EStopPrevState = EStopCurrState;

  currTimeROS = millis();
  if (EStopActive) {
    nh.loginfo("E Stop Active");
    if (servoA.attached()) {
      servoA.detach();
    }
//    if (servoB.attached()) {
//      servoB.detach();
//    }

    if (currTimeROS - prevTimeLEDFast >= LED_BLINK_FAST) {
      prevTimeLEDFast = currTimeROS;
      if (LEDState == LOW) {
        LEDState = HIGH;
      } else {
        LEDState = LOW;
      }
    }
    digitalWrite(calLEDPin, LEDState);

    currTimeROS = millis();
    if (currTimeROS - prevTimeROS > 1) {
      cmdArrayFloat[0] = 0;
      cmdArrayFloat[1] = 0;
      cmdArrayFloat[2] = 0;
      cmdArrayFloat[3] = 0;
      cmdArrayFloat[4] = 0;
      if (ROS_MODE) {
        cmdMsgArrayFloat.data = cmdArrayFloat;
        cmdMsgArrayFloat.data_length = 5;
        FootInterface.publish(&cmdMsgArrayFloat);
        nh.spinOnce();
      }
      prevTimeROS = currTimeROS;
    }

  } else {
    if(footLifted){
//    if (withdrawalPressure < FSR_SLOW_THRESHOLD || insertionPressure < FSR_SLOW_THRESHOLD){
      PIDA_MODE = false;
//      PIDB_MODE = false;
    }else{
      PIDA_MODE = true;
//      PIDB_MODE = false;
    }

//    if(footLifted){
//        PIDB_MODE = false;
//    }
//    else{
//      PIDB_MODE = false;
//    }
    // Read Membrane Pot Val
    membranePotValPrev = membranePotVal;
    membranePotVal = analogRead(membranePotPin);
    membraneAvg = (membranePotValPrev + membranePotVal) / 2;
    
    // Read and map encoder value
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_CS, LOW);
    int encoderVal = 0;
    for (int i = 0; i < 16; i++) {
      digitalWrite(PIN_CLOCK, LOW);
      digitalWrite(PIN_CLOCK, HIGH);
      byte bEnc = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
      encoderVal += bEnc * pow(2, 10 - (i + 1));
    }

    // Status bits, not written or read
    for (int i = 0; i < 6; i++) {
      digitalWrite(PIN_CLOCK, LOW);
      digitalWrite(PIN_CLOCK, HIGH);
    }

    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);

    if (encoderVal - 980 < 0) {
      encoderValMapped = 1023 - 980 + encoderVal + 1;  // handle wrap around from 1023 - 0 based on encoder mounting position and max travel (less < 1 full rotation)
    } else {
      encoderValMapped = encoderVal - 980;
    }

    // Clamp Limits
    if (encoderValMapped > 874 && encoderValMapped <= 963) {
      encoderValMapped = 874;
      clampLeft = true;
    } else {
      clampLeft = false;
    }

    if (encoderValMapped >= 964 && encoderValMapped <= 1023) {
      encoderValMapped = 0;
      clampRight = true;
    } else {
      clampRight = false;
    }
    
    // PID Control Mode Parameters
    if (PIDA_MODE) {
      // Compliance
      maxSpeedValA = 200;
      minSpeedValA = 0;
      PIDTargetA = (thresholdLowA + thresholdHighA)/2;
      kpA = 1;
      kiA = 0;
      kdA = 0;
    } else {
      // Snap to position
      maxSpeedValA = 200;
      minSpeedValA = 50;
      PIDTargetA = (ENCODER_HOME_LOWER + ENCODER_HOME_UPPER) / 2;
      kpA = 1;
      kiA = 0;
      kdA = 0;
    }

    if (PIDB_MODE) {
      maxSpeedValB = 70;
      minSpeedValB = 0;
      PIDTargetB = (thresholdLowB + thresholdHighB)/2;
      kpB = 0.3;
      kiB = 0;
      kdB = 0;
    } else {
      maxSpeedValB = 60;
      minSpeedValB = 0;
      PIDTargetB = (HOME_MEMBRANE_LOWER + HOME_MEMBRANE_UPPER) / 2;
      kpB = 0.4;
      kiB = 0.1;
      kdB = 0;
    }

    // Re-Attach servo if E-stop trigged
    if (!servoA.attached()) {
      servoA.attach(ServoAPin);
    }
//    if (!servoB.attached()) {
//      servoB.attach(ServoBPin);
//    }

    // ------------------- //
    // FSR CALIBRATION
    // ------------------- //
    calSwitch = digitalRead(calSwitchPin);

    if (calSwitch) {

      // SERVO A
      // PID to drive servo to home position
      combinedValA = encoderValMapped;
      errorA = PIDTargetA - combinedValA;

      PIDA_MODE = false;
      PIDB_MODE = false;
      
      if (encoderValMapped > (ENCODER_HOME_LOWER + 10) && encoderValMapped < (ENCODER_HOME_UPPER - 10)) {
        encoderValMapped = PIDTargetA;
        eIntegralA = 0;
      }

      currTA = micros();
      deltaTA = ((float)(currTA - prevTA)) / 1.0e6;
      prevTA = currTA;
      dedtA = (errorA - ePrevA) / deltaTA;
      eIntegralA = eIntegralA + errorA * deltaTA;

      // integral wind up
      if ((kiA * eIntegralA) > maxSpeedValA) {
        eIntegralA = maxSpeedValA;
      }

      if ((kiA * eIntegralA) < -maxSpeedValA) {
        eIntegralA = -maxSpeedValA;
      }

      uA = kpA * errorA + kdA * dedtA + kiA * eIntegralA;
      ePrevA = errorA;

      // Limit control signal
      if (uA > maxSpeedValA) {
        uA = maxSpeedValA;
      }

      if (uA < -maxSpeedValA) {
        uA = -maxSpeedValA;
      }

      controlSignalA = (int)(uA) + SERVO_RESTING_A;
      controlValAFiltered = (int)ControlValAFilter.filter(controlSignalA);
      servoA.writeMicroseconds(controlValAFiltered);
      Serial.println(controlValAFiltered);

      // SERVO B
      // PID to drive servo to home position
      combinedValB = membraneAvg;
      errorB = PIDTargetB - combinedValB;

      if (membraneAvg > (HOME_MEMBRANE_LOWER + 10) && membraneAvg < (HOME_MEMBRANE_UPPER - 10)) {
        membraneAvg = PIDTargetB;
        eIntegralB = 0;
      }

      currTB = micros();
      deltaTB = ((float)(currTB - prevTB)) / 1.0e6;
      prevTB = currTB;
      dedtB = (errorB - ePrevB) / deltaTB;
      eIntegralB = eIntegralB + errorB * deltaTB;

      // integral wind up
      if ((kiB * eIntegralB) > maxSpeedValB) {
        eIntegralB = maxSpeedValB;
      }

      if ((kiB * eIntegralB) < -maxSpeedValB) {
        eIntegralB = -maxSpeedValB;
      }

      uB = kpB * errorB + kdB * dedtB + kiB * eIntegralB;
      ePrevB = errorB;

      // Limit control signal
      if (uB > maxSpeedValB) {
        uB = maxSpeedValB;
      }

      if (uB < -maxSpeedValB) {
        uB = -maxSpeedValB;
      }

      controlSignalB = (int)(uB) + SERVO_RESTING_B;
      controlValBFiltered = (int)ControlValBFilter.filter(controlSignalB);
//      servoB.writeMicroseconds(controlValBFiltered);

      // FSR Calibration
      digitalWrite(calLEDPin, HIGH);
      A0val = analogRead(A0);
      A1val = analogRead(A1);
      A2val = analogRead(A2);
      A3val = analogRead(A3);
      A4val = analogRead(A4);
      A5val = analogRead(A5);

      if (firstLoopCal) {
        // Reset values
        maxVal[0] = 0;
        maxVal[1] = 0;
        maxVal[2] = 0;
        maxVal[3] = 0;
        maxVal[4] = 0;
        maxVal[5] = 0;
      }
      if (A0val > maxVal[0]) {
        maxVal[0] = A0val;
      }

      if (A1val > maxVal[1]) {
        maxVal[1] = A1val;
      }

      if (A2val > maxVal[2]) {
        maxVal[2] = A2val;
      }

      if (A3val > maxVal[3]) {
        maxVal[3] = A3val;
      }

      if (A4val > maxVal[4]) {
        maxVal[4] = A4val;
      }

      if (A5val > maxVal[5]) {
        maxVal[5] = A5val;
      }

      // Insertion Sensors
      if (maxVal[0] > maxVal[1]) {
        tempSort = maxVal[0];
        tempPin = 0;
      } else {
        tempSort = maxVal[1];
        tempPin = 1;
      }

      if (maxVal[2] > tempSort) {
        sensorNumInsertion = 2;
      } else {
        sensorNumInsertion = tempPin;
      }

      // Withdrawal Sensors
      if (maxVal[3] > maxVal[4]) {
        tempSort = maxVal[3];
        tempPin = 3;
      } else {
        tempSort = maxVal[4];
        tempPin = 4;
      }

      if (maxVal[5] > tempSort) {
        sensorNumWithdrawal = 5;
      } else {
        sensorNumWithdrawal = tempPin;
      }

      firstLoopCal = false;
    } else {
      bool PIDB_MODE = true;
      // ------------------- //
      // Normal Operation (calibration mode off)
      // ------------------- //
      digitalWrite(calLEDPin, LOW);
      firstLoopCal = true;
      switch (sensorNumInsertion) {
        case 0:
          insertionPin = A0;  // sensor removed
          break;
        case 1:
          insertionPin = A1;
          break;
        case 2:
          insertionPin = A2;
          break;
        default:
          insertionPin = A1;
      }

      switch (sensorNumWithdrawal) {
        case 3:
          withdrawalPin = A3;
          break;
        case 4:
          withdrawalPin = A4;
          break;
        case 5:
          withdrawalPin = A5;
          break;
          break;
        default:
          withdrawalPin = A3;
      }

      // ------------------- //
      // INSERTION
      // ------------------- //
      // Read Sensor Values
      insertionPressure = analogRead(insertionPin);
      withdrawalPressure = analogRead(withdrawalPin);

      // Check values against threshold
      if (insertionPressure < FSR_SLOW_THRESHOLD) {
        currStateIns = false;
      } else {
        currStateIns = true;
      }

      if (withdrawalPressure < FSR_SLOW_THRESHOLD) {
        currStateWith = false;
      } else {
        currStateWith = true;
      }

      // Detect if the foot is lifted off the pedal
      // Flash LED if foot lifed
      if (withdrawalPressure < FSR_SLOW_THRESHOLD && insertionPressure < FSR_SLOW_THRESHOLD) {
        footLifted = true;

        if (currTimeROS - prevTimeLEDSlow >= LED_BLINK_SLOW) {
          prevTimeLEDSlow = currTimeROS;
          if (LEDState == LOW) {
            LEDState = HIGH;
          } else {
            LEDState = LOW;
          }
        }
        digitalWrite(calLEDPin, LEDState);
      }

      // Record State Change Times (when sensor detects foot lift or tap on insertion/withdrawal sensors)
      if (currStateIns != prevStateIns && currStateWith) {
        if (countTapIns == 0 && !currStateIns) {
          startTimeIns = millis();
          countTapIns += 1;
          timeLastIns = startTimeIns;
        } else if ((footLifted || insertionStopped) && currStateIns) {
          countTapIns = 0;
          footLifted = false;
          insertionStopped = false;
        } else if (countTapIns > 0) {
          timeAIns = millis();

          if (abs(timeAIns - timeLastIns) > MIN_TAP_TIME) {
            countTapIns += 1;
          }
          if (abs(timeAIns - startTimeIns) > MAX_TAP_TIME) {
            countTapIns = 1;
            startTimeIns = timeAIns;
          }
          timeLastIns = timeAIns;
        }
      }

      // Insertion actuation
      if (countTapIns == 4) {
        while (insertionPressure > FSR_SLOW_THRESHOLD) {
          withdrawalPressure = analogRead(withdrawalPin);
          delay(5);

          if (withdrawalPressure < FSR_FAST_THRESHOLD) {
            if (insertionRampCounter < RAMP_THRESHOLD_INS && cmdArray[2] != 3) {
              insertionRampCounter += 1;
              cmdArray[2] = 2;
              nh.loginfo("insertion ramp");
            } else if (insertionRampCounter >= RAMP_THRESHOLD_INS) {
              insertionRampCounter = 0;
              cmdArray[2] = 3;
              nh.loginfo("insertion fast");
            }

          } else {
            cmdArray[2] = 1;
            nh.loginfo("insertion slow");
          }

          if (!vibMotor1Active) {
            prevTimeVibEnc = currTimeROS;
          }
          if (!vibrationInhibitIns) {
            if (currTimeROS - prevTimeVibEnc <= VIBRATION_DURATION) {
              digitalWrite(vibrationMotor1Pin, HIGH);
              vibMotor1Active = true;

            } else {
              digitalWrite(vibrationMotor1Pin, LOW);
              vibMotor1Active = false;
              vibrationInhibitIns = true;
            }
          } else {
            digitalWrite(vibrationMotor1Pin, LOW);
          }

          insertionPressure = analogRead(insertionPin);
          delay(5);

          currTimeROS = millis();
          if (currTimeROS - prevTimeROS > 1) {
            const float MIN_SPEED_PITCH = 0.002;
            const float MAX_SPEED_PITCH = 0.007;

            // Pitch (Left)
            if (cmdArray[2] == 1) {
              cmdArrayFloat[2] = MIN_SPEED_PITCH;
            } else if (cmdArray[2] == 3) {
              cmdArrayFloat[2] = MAX_SPEED_PITCH;
            } else if (cmdArray[2] == 2) {
              cmdArrayFloat[2] += 0.0004;
              if (cmdArrayFloat[2] > MAX_SPEED_PITCH) {
                cmdArrayFloat[2] = MAX_SPEED_PITCH;
              }
            }
            // Pitch (Right)
            else if (cmdArray[3] == 1) {
              cmdArrayFloat[2] = -MIN_SPEED_PITCH;
            } else if (cmdArray[3] == 3) {
              cmdArrayFloat[2] = -MAX_SPEED_PITCH;
            } else if (cmdArray[3] == 2) {
              cmdArrayFloat[2] -= 0.0004;
              if (cmdArrayFloat[2] < -MAX_SPEED_PITCH) {
                cmdArrayFloat[2] = -MAX_SPEED_PITCH;
              }
            } else {
              cmdArrayFloat[2] = 0;
            }

            if (ROS_MODE) {
              cmdMsgArrayFloat.data = cmdArrayFloat;
              cmdMsgArrayFloat.data_length = 5;
              FootInterface.publish(&cmdMsgArrayFloat);
              nh.spinOnce();
            }
            prevTimeROS = currTimeROS;
          }
        }
        cmdArray[2] = 0;
        cmdArrayFloat[2] = 0;
        insertionStopped = true;
        insertionRampCounter = 0;
        vibrationInhibitIns = false;
      }


      prevStateIns = currStateIns;

      // ------------------- //
      // WITHDRAWAL
      // ------------------- //
      // Record State Change Times
      if (currStateWith != prevStateWith && currStateIns) {
        if (countTapWith == 0 && !currStateWith) {
          startTimeWith = millis();
          countTapWith += 1;
          timeLastWith = startTimeWith;
        } else if ((footLifted || withdrawalStopped) && currStateIns) {
          countTapWith = 0;
          footLifted = false;
          withdrawalStopped = false;
        } else if (countTapWith > 0) {
          timeAWith = millis();

          if (abs(timeAWith - timeLastWith) > MIN_TAP_TIME) {
            countTapWith += 1;
          }
          if (abs(timeAWith - startTimeWith) > MAX_TAP_TIME) {
            countTapWith = 1;
            startTimeWith = timeAWith;
          }
          timeLastWith = timeAWith;
        }
      }

      // withdrawal actuation
      if (countTapWith == 4) {
        while (withdrawalPressure > FSR_SLOW_THRESHOLD) {

          insertionPressure = analogRead(insertionPin);
          delay(5);

          if (insertionPressure < FSR_FAST_THRESHOLD) {
            if (withdrawalRampCounter < RAMP_THRESHOLD_INS && cmdArray[3] != 3) {
              withdrawalRampCounter += 1;
              cmdArray[3] = 2;
              nh.loginfo("withdrawal ramp");
            } else if (withdrawalRampCounter >= RAMP_THRESHOLD_INS) {
              withdrawalRampCounter = 0;
              cmdArray[3] = 3;
              nh.loginfo("withdrawal fast");
            }

          } else {
            cmdArray[3] = 1;
            nh.loginfo("withdrawal slow");
          }

          if (!vibMotor1Active) {
            prevTimeVibEnc = currTimeROS;
          }
          if (!vibrationInhibitIns) {
            if (currTimeROS - prevTimeVibEnc <= VIBRATION_DURATION) {
              digitalWrite(vibrationMotor1Pin, HIGH);
              vibMotor1Active = true;

            } else {
              digitalWrite(vibrationMotor1Pin, LOW);
              vibMotor1Active = false;
              vibrationInhibitIns = true;
            }
          } else {
            digitalWrite(vibrationMotor1Pin, LOW);
          }

          withdrawalPressure = analogRead(withdrawalPin);
          delay(5);

          // While loop is blocking - publish inside while loop (to be fixed if time)
          currTimeROS = millis();
          if (currTimeROS - prevTimeROS > 1) {
            const float MIN_SPEED_PITCH = 0.002;
            const float MAX_SPEED_PITCH = 0.007;

            // Pitch (Left)
            if (cmdArray[2] == 1) {
              cmdArrayFloat[2] = MIN_SPEED_PITCH;
            } else if (cmdArray[2] == 3) {
              cmdArrayFloat[2] = MAX_SPEED_PITCH;
            } else if (cmdArray[2] == 2) {
              cmdArrayFloat[2] += 0.0004;
              if (cmdArrayFloat[2] > MAX_SPEED_PITCH) {
                cmdArrayFloat[2] = MAX_SPEED_PITCH;
              }
            }
            // Pitch (Right)
            else if (cmdArray[3] == 1) {
              cmdArrayFloat[2] = -MIN_SPEED_PITCH;
            } else if (cmdArray[3] == 3) {
              cmdArrayFloat[2] = -MAX_SPEED_PITCH;
            } else if (cmdArray[3] == 2) {
              cmdArrayFloat[2] -= 0.0004;
              if (cmdArrayFloat[2] < -MAX_SPEED_PITCH) {
                cmdArrayFloat[2] = -MAX_SPEED_PITCH;
              }
            } else {
              cmdArrayFloat[2] = 0;
            }

            if (ROS_MODE) {
              cmdMsgArrayFloat.data = cmdArrayFloat;
              cmdMsgArrayFloat.data_length = 5;
              FootInterface.publish(&cmdMsgArrayFloat);
              nh.spinOnce();
            }
            prevTimeROS = currTimeROS;
          }
        }
        withdrawalStopped = true;
        cmdArray[3] = 0;
        cmdArrayFloat[2] = 0;
        withdrawalRampCounter = 0;
        vibrationInhibitIns = false;
      }

      prevStateWith = currStateWith;

      // ------------------- //
      // ROLL
      // ------------------- //
      // Read Roll Hall Effect Sensor Raw Values
      rollAVal = analogRead(A8);
      rollBVal = analogRead(A9);

      // Roll Hall Effect Actuation Commands (Sensor A)
      // Fast
      if (rollAVal > ROLL_THRESHOLD_FAST) {
        if (rollARampCounter < RAMP_COUNT_THRESHOLD && cmdArray[0] != 3) {
          rollARampCounter += 1;
          cmdArray[0] = 2;
        } else if (rollARampCounter >= RAMP_COUNT_THRESHOLD) {
          rollARampCounter = 0;
          cmdArray[0] = 3;
        }

        // Slow
      } else if (rollAVal > ROLL_THRESHOLD_SLOW) {
        cmdArray[0] = 1;
      } else {
        cmdArray[0] = 0;
        rollARampCounter = 0;
      }

      // Roll Hall Effect Actuation Commands (Sensor B)
      // Fast
      if (rollBVal > ROLL_THRESHOLD_FAST) {
        if (rollBRampCounter < RAMP_COUNT_THRESHOLD && cmdArray[1] != 3) {
          rollBRampCounter += 1;
          cmdArray[1] = 2;
        } else if (rollBRampCounter >= RAMP_COUNT_THRESHOLD) {
          rollBRampCounter = 0;
          cmdArray[1] = 3;
        }

        // Slow
      } else if (rollBVal > ROLL_THRESHOLD_SLOW) {
        cmdArray[1] = 1;
      } else {
        cmdArray[1] = 0;
        rollBRampCounter = 0;
      }

      // ------------------- //
      // PITCH (MEMBRANE POT) - replaced with Bourns ACE128 encoder
      // ------------------- //

      // Forward
      if (membraneAvg > PITCH_FORWARD_FAST) {
        if (pitchRampCounter < PITCH_RAMP_COUNT_THRESHOLD && cmdArray[4] != 3) {
          // Ramp
          pitchRampCounter += 1;
          cmdArray[4] = 2;
        }
        // Fast
        else if (pitchRampCounter >= PITCH_RAMP_COUNT_THRESHOLD) {
          pitchRampCounter = 0;
          cmdArray[4] = 3;
        }

        // Slow
      } else if (membraneAvg > PITCH_FORWARD_SLOW || (cmdArray[4] == 1 && membraneAvg > HOME_MEMBRANE_UPPER)) {
        cmdArray[4] = 1;
        vibrationInhibitMembrane = false;

        // Backward
      } else if (membraneAvg < PITCH_BACKWARD_FAST) {
        if (pitchRampCounter < PITCH_RAMP_COUNT_THRESHOLD && cmdArray[5] != 3) {
          // Ramp
          pitchRampCounter += 1;
          cmdArray[5] = 2;
        }
        // Fast
        else if (pitchRampCounter >= PITCH_RAMP_COUNT_THRESHOLD) {
          pitchRampCounter = 0;
          cmdArray[5] = 3;
        }

      } else if (membraneAvg < PITCH_BACKWARD_SLOW || (cmdArray[5] == 1 && membraneAvg < HOME_MEMBRANE_LOWER)) {
        cmdArray[5] = 1;
        vibrationInhibitMembrane = false;
        // Off
      } else {
        cmdArray[5] = 0;
        cmdArray[4] = 0;
        pitchRampCounter = 0;
      }

      //    nh.loginfo("Membrane pot");1
//       itoa(membranePotVal, tempROSLogging, 10);
//       nh.loginfo(tempROSLogging);

      // Active Vibration Motor if at home position
      if (membraneAvg > HOME_MEMBRANE_LOWER && membranePotVal < HOME_MEMBRANE_UPPER) {

        if (!vibMotor1Active) {
          prevTimeVibMem = currTimeROS;
        }
        if (!vibrationInhibitMembrane) {
          if (currTimeROS - prevTimeVibMem <= VIBRATION_DURATION) {
            digitalWrite(vibrationMotor1Pin, HIGH);
            vibMotor1Active = true;

          } else {
            digitalWrite(vibrationMotor1Pin, LOW);
            vibMotor1Active = false;
            vibrationInhibitMembrane = true;
          }
        } else {
          digitalWrite(vibrationMotor1Pin, LOW);
        }
      } else {
        digitalWrite(vibrationMotor1Pin, LOW);
      }


      // ------------------- //
      // YAW (ENCODER)
      // ------------------- //



      // YAW_LEFT_FAST = 800;
      // YAW_LEFT_SLOW = 700;
      // YAW_RIGHT_SLOW = 450;
      // YAW_RIGHT_FAST = 350;

      // Left
      if (encoderValMapped > YAW_LEFT_FAST) {
        if (yawRampCounter < YAW_RAMP_COUNT_THRESHOLD && cmdArray[6] != 3) {
          // Ramp
          yawRampCounter += 1;
          cmdArray[6] = 2;
        }
        // Fast
        else if (yawRampCounter >= YAW_RAMP_COUNT_THRESHOLD) {
          yawRampCounter = 0;
          cmdArray[6] = 3;
        }

        // Slow
      } else if (encoderValMapped > YAW_LEFT_SLOW || (cmdArray[6] == 1 && encoderValMapped > ENCODER_HOME_UPPER)) {
        cmdArray[6] = 1;
        vibrationInhibitEncoder = false;

        // Right
      } else if (encoderValMapped < YAW_RIGHT_FAST) {
        if (yawRampCounter < YAW_RAMP_COUNT_THRESHOLD && cmdArray[7] != 3) {
          // Ramp
          yawRampCounter += 1;
          cmdArray[7] = 2;
        }
        // Fast
        else if (yawRampCounter >= YAW_RAMP_COUNT_THRESHOLD) {
          yawRampCounter = 0;
          cmdArray[7] = 3;
        }

      } else if (encoderValMapped < YAW_RIGHT_SLOW || (cmdArray[7] == 1 && encoderValMapped < ENCODER_HOME_LOWER)) {
        cmdArray[7] = 1;
        vibrationInhibitEncoder = false;
        // Off
      } else {
        cmdArray[7] = 0;
        cmdArray[6] = 0;
        yawRampCounter = 0;
      }

      // nh.loginfo("Encoder Val Mapped");
      //      itoa(encoderValMapped, tempROSLogging, 10);
      //      nh.loginfo(tempROSLogging);

      // nh.loginfo("Encoder Val");
      //    itoa(encoderVal, tempROSLogging, 10);
      //    nh.loginfo(tempROSLogging);

      // Active Vibration Motor if at home position
      if (encoderValMapped > ENCODER_HOME_LOWER && encoderValMapped < ENCODER_HOME_UPPER) {

        if (!vibMotor2Active) {
          prevTimeVibEnc = currTimeROS;
        }
        if (!vibrationInhibitEncoder) {
          if (currTimeROS - prevTimeVibEnc <= VIBRATION_DURATION) {
            digitalWrite(vibrationMotor2Pin, HIGH);
            vibMotor2Active = true;

          } else {
            digitalWrite(vibrationMotor2Pin, LOW);
            vibMotor2Active = false;
            vibrationInhibitEncoder = true;
          }
        } else {
          digitalWrite(vibrationMotor2Pin, LOW);
        }
      } else {
        digitalWrite(vibrationMotor2Pin, LOW);
      }

      // ------------------- //
      // COUPLED CONTROL MODE
      // ------------------- //
      // If decoupled mode is enabled, the furthest direction from home takes precedence

      if (DECOUPLED_CONTROL) {
        if (abs(encoderValMapped - (ENCODER_HOME_LOWER + ENCODER_HOME_UPPER) / 2) > abs(membraneAvg - (HOME_MEMBRANE_LOWER + HOME_MEMBRANE_UPPER) / 2)) {
          cmdArray[4] = 0;
          cmdArray[5] = 0;
        } else {
          cmdArray[6] = 0;
          cmdArray[7] = 0;
        }
      }

      // ------------------- //
      // SHARED CONTROL
      // ------------------- //
      // Read shared control hall effect sensor
      sharedControlVal = analogRead(A10);

      if (sharedControlVal > SHARED_CONTROL_ACTIVATE) {
        // Enforce 3 second minimum window between stage changes
        // This provides a debounce so the state does not keep alternating when the button is pressed
        if (currTimeROS - prevControlStateTime > 3000) {
          sharedControlActive = !sharedControlActive;  // Change state if ON turn OFF and vice versa
          cmdArray[8] = int(sharedControlActive);
          prevControlStateTime = currTimeROS;
        }
      }

      // ------------------- //
      //Servo A PID
      // ------------------- //
      // PID code adapted from: https://curiores.com/positioncontrol

      servoAHall1 = analogRead(servoAHall1Pin);
      servoAHall2 = analogRead(servoAHall2Pin);

      if (!PIDA_MODE) {
        combinedValA = encoderValMapped;
        errorA = PIDTargetA - combinedValA;
      } else {
        combinedValA = (servoAHall1+servoAHall2)/2;
        errorA = combinedValA - PIDTargetA;
      }

      if (encoderValMapped > (ENCODER_HOME_LOWER + 10) && encoderValMapped < (ENCODER_HOME_UPPER - 10)) {
        encoderValMapped = PIDTargetA;
        eIntegralA = 0;
      }

      currTA = micros();
      deltaTA = ((float)(currTA - prevTA)) / 1.0e6;
      prevTA = currTA;
      dedtA = (errorA - ePrevA) / deltaTA;
      eIntegralA = eIntegralA + errorA * deltaTA;

      // integral wind up
      if ((kiA * eIntegralA) > maxSpeedValA) {
        eIntegralA = maxSpeedValA;
      }

      if ((kiA * eIntegralA) < -maxSpeedValA) {
        eIntegralA = -maxSpeedValA;
      }

      uA = kpA * errorA + kdA * dedtA + kiA * eIntegralA;
      ePrevA = errorA;

      // Limit control signal
      if (uA > maxSpeedValA) {
        uA = maxSpeedValA;
      }

      if (uA < -maxSpeedValA) {
        uA = -maxSpeedValA;
      }

      controlSignalA = (int)(uA) + SERVO_RESTING_A;
      controlValAFiltered = (int)ControlValAFilter.filter(controlSignalA);
      combinedValAFiltered = (int)CombinedValAFilter.filter(combinedValA); // tbc if needed?

            servoA.writeMicroseconds(controlValAFiltered);
//
//           if (!ROS_MODE) {
//        //        Serial.print(0);
//        //        Serial.print(",");
//
//        Serial.print(servoAHall1);
//        Serial.print(",");
//        Serial.print(servoAHall2);
//        Serial.print(",");
////        Serial.print(400);
////        Serial.print(",");
////        Serial.print(450);
////        Serial.print(",");
////        Serial.print(500);
////        Serial.print(",");
//        Serial.print(combinedValAFiltered);
////        Serial.print(",");
////        Serial.print(SERVO_RESTING_B);
//        Serial.print(",");
//        Serial.println(controlValAFiltered);
//      }

      
      // ------------------- //
      //Servo B PID
      // ------------------- //
      servoBHall1 = analogRead(servoBHall1Pin);
      servoBHall2 = analogRead(servoBHall2Pin);

      if (!PIDB_MODE) {
        combinedValB = membraneAvg;
        errorB = PIDTargetB - combinedValB;
      } else {
        combinedValB = (servoBHall1+servoBHall2)/2;
//         errorB = PIDTargetB - combinedValB;
        errorB = combinedValB - PIDTargetB;
      }

      if (membraneAvg > (HOME_MEMBRANE_LOWER + 10) && membraneAvg < (HOME_MEMBRANE_UPPER - 10)) {
        membraneAvg = PIDTargetB;
        eIntegralB = 0;
      }

      currTB = micros();
      deltaTB = ((float)(currTB - prevTB)) / 1.0e6;
      prevTB = currTB;
      dedtB = (errorB - ePrevB) / deltaTB;
      eIntegralB = eIntegralB + errorB * deltaTB;

      // integral wind up
      if ((kiB * eIntegralB) > maxSpeedValB) {
        eIntegralB = maxSpeedValB;
      }

      if ((kiB * eIntegralB) < -maxSpeedValB) {
        eIntegralB = -maxSpeedValB;
      }

      uB = kpB * errorB + kdB * dedtB + kiB * eIntegralB;
      ePrevB = errorB;

      // Limit control signal
      if (uB > maxSpeedValB) {
        uB = maxSpeedValB;
      }

      if (uB < -maxSpeedValB) {
        uB = -maxSpeedValB;
      }

      controlSignalB = (int)(uB) + SERVO_RESTING_B;
      controlValBFiltered = (int)ControlValBFilter.filter(controlSignalB);
      combinedValBFiltered = (int)CombinedValBFilter.filter(combinedValB); // tbc if needed?

        if (!ROS_MODE) {
        //        Serial.print(0);
        //        Serial.print(",");

        Serial.print(servoBHall1);
        Serial.print(",");
        Serial.print(servoBHall2);
        Serial.print(",");
//        Serial.print(400);
//        Serial.print(",");
//        Serial.print(450);
//        Serial.print(",");
//        Serial.print(500);
//        Serial.print(",");
        Serial.print(combinedValBFiltered);
//        Serial.print(",");
//        Serial.print(SERVO_RESTING_B);
        Serial.print(",");
        Serial.println(controlValBFiltered);
      }
      

//      servoB.writeMicroseconds(controlValBFiltered);

      // ------------------- //
      //Publish ROS Topic (array)
      // ------------------- //
      // itoa(encoderValMapped, tempROSLogging, 10);
      // nh.loginfo(tempROSLogging);

      if (currTimeROS - prevTimeROS > 1) {
        // CmdArray to CmdArrayFloat mapping
        // X (6,7)
        // Y (4,5)
        // Pitch (2,3)
        // Rotation (0,1)

        // Y Translation (Down)
        if (cmdArray[4] == 1) {
          cmdArrayFloat[0] = -MIN_SPEED_Y;
        } else if (cmdArray[4] == 3) {
          cmdArrayFloat[0] = -MAX_SPEED_Y;
        } else if (cmdArray[4] == 2) {
          cmdArrayFloat[0] -= 0.0002;
          if (cmdArrayFloat[0] < -MAX_SPEED_Y) {
            cmdArrayFloat[0] = -MAX_SPEED_Y;
          }
        }

        // Y Translation (Up)
        else if (cmdArray[5] == 1) {
          cmdArrayFloat[0] = MIN_SPEED_Y;
        } else if (cmdArray[5] == 3) {
          cmdArrayFloat[0] = MAX_SPEED_Y;
        } else if (cmdArray[5] == 2) {
          cmdArrayFloat[0] += 0.0002;
          if (cmdArrayFloat[0] > MAX_SPEED_Y) {
            cmdArrayFloat[0] = MAX_SPEED_Y;
          }
        } else {
          cmdArrayFloat[0] = 0;
        }

        // X Translation (Left)
        if (cmdArray[6] == 1) {
          cmdArrayFloat[1] = MIN_SPEED_X;
        } else if (cmdArray[6] == 3) {
          cmdArrayFloat[1] = MAX_SPEED_X;
        } else if (cmdArray[6] == 2) {
          cmdArrayFloat[1] += 0.0002;
          if (cmdArrayFloat[1] > MAX_SPEED_X) {
            cmdArrayFloat[1] = MAX_SPEED_X;
          }
        }

        // X Translation (Right)
        else if (cmdArray[7] == 1) {
          cmdArrayFloat[1] = -MIN_SPEED_X;
        } else if (cmdArray[7] == 3) {
          cmdArrayFloat[1] = -MAX_SPEED_X;
        } else if (cmdArray[7] == 2) {
          cmdArrayFloat[1] -= 0.0002;
          if (cmdArrayFloat[1] < MAX_SPEED_X) {
            cmdArrayFloat[1] = -MAX_SPEED_X;
          }
        } else {
          cmdArrayFloat[1] = 0;
        }

        // Rotation (Left)
        if (cmdArray[0] == 1) {
          cmdArrayFloat[3] = -MIN_SPEED_ROTATION;
        } else if (cmdArray[0] == 3) {
          cmdArrayFloat[3] = -MAX_SPEED_ROTATION;
        } else if (cmdArray[0] == 2) {
          cmdArrayFloat[3] -= 0.004;
          if (cmdArrayFloat[3] < -MAX_SPEED_ROTATION) {
            cmdArrayFloat[3] = -MAX_SPEED_ROTATION;
          }
        }
        // Rotation (Right)
        else if (cmdArray[1] == 1) {
          cmdArrayFloat[3] = MIN_SPEED_ROTATION;
        } else if (cmdArray[1] == 3) {
          cmdArrayFloat[3] = MAX_SPEED_ROTATION;
        } else if (cmdArray[1] == 2) {
          cmdArrayFloat[3] += 0.004;
          if (cmdArrayFloat[3] > MAX_SPEED_ROTATION) {
            cmdArrayFloat[3] = MAX_SPEED_ROTATION;
          }
        } else {
          cmdArrayFloat[3] = 0;
        }
        // itoa(rollAVal, tempROSLogging, 10);
        // nh.loginfo(tempROSLogging);
        // itoa(cmdArray[0] , tempROSLogging, 10);
        // nh.loginfo(tempROSLogging);

        if (ROS_MODE) {
          cmdMsgArrayFloat.data = cmdArrayFloat;
          cmdMsgArrayFloat.data_length = 5;
          FootInterface.publish(&cmdMsgArrayFloat);

          // nh.loginfo("Shared control val");
          //itoa(membranePotVal, tempROSLogging, 10);
          //nh.loginfo(tempROSLogging);
          // nh.loginfo("shared control state");
          // itoa(sharedControlActive, tempROSLogging, 10);
          // nh.loginfo(tempROSLogging);

          nh.spinOnce();
        }
        prevTimeROS = currTimeROS;
      }

      delay(1);  // for analog read stability
    }
  }
}
