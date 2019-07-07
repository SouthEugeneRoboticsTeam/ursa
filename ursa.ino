#include "mpu.h"
#include "stepper.h"
#include "wifi.h"
#include "ursa.h"
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  mutexReceive = xSemaphoreCreateMutex();

  sprintf(robotSSID, "SERT_URSA_%02d", ROBOT_ID);  // create unique network SSID
  Serial.begin(115200);  // for debugging. Set the serial monitor to the same value or you will see nothing or gibberish.
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  //TODO: disable stepper motors

  setupStepperRMTs();

  PIDA.SetMode(MANUAL);  // PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(1);  // tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(1);
  PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
  PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);

  setupMPU6050();  // this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();  // this function averages some gyro readings so later the readings can be calibrated to zero. This function counts on the robot being still, so the robot needs to be powered on while lying on the ground

  setupWifi();

  setupStepperTimers();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {  // on core 1. the balencing control loop will be here, with the goal of keeping this loop as fast as possible
  readMPU6050();

  if (receivedNewData) {
    if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
      parseDataReceived();
      numBytesToSend = createDataToSend();
      receivedNewData = false;
      xSemaphoreGive(mutexReceive);
    }
  }

  robotEnabled = enable;

  if (abs(pitch) > MAX_TIP) {
    tipped = true;
    robotEnabled = false;
  } else {
    tipped = false;
  }

  if (millis() - lastMessageTimeMillis > WiFiLossDisableIntervalMillis) {
    robotEnabled = false;
  }

  if (robotEnabled) {  // run the following code if the robot is enabled
    digitalWrite(LED_BUILTIN, HIGH);

    if (!wasRobotEnabled) {  // the robot wasn't enabled, but now it is, so this must be the first loop since it was enabled. re set up anything you might want to
      // TODO: turn on stepper motors
      PIDA.SetMode(AUTOMATIC);  // turn on the PID
      PIDS.SetMode(AUTOMATIC);  // turn on the PID
    }

    PIDA.SetTunings(kP_angle, kI_angle, kD_angle);
    PIDS.SetTunings(kP_speed, kI_speed, kD_speed);
    PIDS.Compute();  // compute the PID, it changes the variables you set it up with earlier.
    PIDA.Compute();

    leftMotorSpeed = constrain(motorSpeedVal + turnSpeedVal, -MAX_SPEED, MAX_SPEED);  // combine motor speed and turn to find the speed the left motor should go
    rightMotorSpeed = constrain(motorSpeedVal - turnSpeedVal, -MAX_SPEED, MAX_SPEED);  // combine motor speed and turn to find the speed the right motor should go

    if (abs(leftMotorSpeed) >= 1) {
      timerAlarmWrite(leftStepTimer, 1000000 / leftMotorSpeed, true);  // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(leftStepTimer, 10000000000000000, true);  // don't step
    }

    if (abs(rightMotorSpeed) >= 1) {
      timerAlarmWrite(rightStepTimer, 1000000 / rightMotorSpeed, true);  // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(rightStepTimer, 10000000000000000, true);  // don't step
    }
  } else {  // disable
    digitalWrite(LED_BUILTIN, LOW);
    PIDA.SetMode(MANUAL);
    PIDS.SetMode(MANUAL);
    timerAlarmWrite(leftStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate
    timerAlarmWrite(rightStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    // TODO: turn off stepper motors
  }

  wasRobotEnabled = robotEnabled;
}

void createDataToSend() {
  byte counter = 0;

  addBoolToBuffer(robotEnabled, counter);
  addBoolToBuffer(tipped, counter);
  addByteToBuffer(ROBOT_ID, counter);
  addByteToBuffer(MODEL_NO, counter);
  addFloatToBuffer(pitch, counter);
  addByteToBuffer(voltage, counter);
  addIntToBuffer(leftMotorSpeed, counter);
  addIntToBuffer(rightMotorSpeed, counter);
  addFloatToBuffer(targetPitch, counter);
  addByteToBuffer(numSendAux, counter);  // how many bytes of extra data

  for (int i = 0; i < numSendAux; i++) {
    addByteToBuffer(auxSendArray[i], counter);  // extra data
  }

  return counter;
}

void parseDataReceived() {  // put parse functions here
  byte counter = 0;
  enable = readBoolFromBuffer(counter);
  speedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED, MAX_SPEED);  // 0=back, 100/8=stop, 200=forwards
  turnSpeedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED / 50, MAX_SPEED / 50);  // 0=left, 200=right
  numAuxRecv = readByteFromBuffer(counter);  // how many bytes of control data for extra things

  for (int i = 0; i < numAuxRecv; i++) {
    auxRecvArray[i] = readByteFromBuffer(counter);
  }

  if (readBoolFromBuffer(counter)) {
    kP_angle = readFloatFromBuffer(counter);
    kI_angle = readFloatFromBuffer(counter);
    kD_angle = readFloatFromBuffer(counter);
    kP_speed = readFloatFromBuffer(counter);
    kI_speed = readFloatFromBuffer(counter);
    kD_speed = readFloatFromBuffer(counter);
  }
}
