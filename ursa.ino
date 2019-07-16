#include <PID_v1.h>
#include <Wire.h>  // scl=22 sda=21
#include "driver/rmt.h"
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define ROBOT_ID 0  // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 0  // unique configuration of robot which can be used to identify additional features
#define WiFiLossDisableIntervalMillis 100  // if no data packet has been recieved for this number of milliseconds, the robot disables to prevent running away
#define DACUnitsPerVolt 123.4  // Use to calibrate voltage read through voltage divider. Divide analogRead value by this constant to get voltage. Analog read is from 0 to 4095 corresponding to 0 to 3.3 volts.
float COMPLEMENTARY_FILTER_CONSTANT = .9997;  // higher = more gyro based, lower=more accelerometer based
int MAX_SPEED = 4000;  // max speed (in steps/sec) that the motors can run at
float MAX_TIP = 60;  // max angle in degrees the robot will attempt to recover from -- if passed, robot will disable
float DRIVE_SPEED_SCALER = .85;  // what proportion of MAX_SPEED the robot's target driving speed can be-some extra speed must be kept in reserve to remain balanced
float TURN_SPEED_SCALER = .05;  // what proportion of MAX_SPEED can be given differently to each wheel in order to turn-controls maximum turn rate
float pitchOffset = 0.000;  // subtracted from the output in readMPU6050 so that zero pitch can correspond to balanced, not that the control loop cares. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not otherwise respond to perfectly balanced.

// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
#define LEFT_STEP_PIN GPIO_NUM_32
#define LEFT_DIR_PIN GPIO_NUM_33
#define RIGHT_STEP_PIN GPIO_NUM_34
#define RIGHT_DIR_PIN GPIO_NUM_35
#define ENS_PIN GPIO_NUM_23  // pin wired to both motor driver chips' ENable pins, to turn on and off motors
#define LED_BUILTIN GPIO_NUM_2
#define VOLTAGE_PIN GPIO_NUM_36  // ADC1 CH0

#define movementThreshold 25
#define movementMeasurements 15

#define maxWifiRecvBufSize 50  // max number of bytes to receive
#define maxWifiSendBufSize 50  // max number of bytes to send

byte voltage = 0;  // 0v=0 13v=255

// since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexReceive;  // used to check whether receiving tasks can safely change shared variables

boolean robotEnabled = false;     // enable outputs?
boolean wasRobotEnabled = false;  // to know if robotEnabled has changed
boolean enable = false;           // is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;

double targetPitch = 0.000;  // what angle the balencing control loop should aim for the robot to be at, the output of the speed control loop
volatile int leftMotorSpeed = 0;  // stepper ticks per second that the left motor is currently doing "volatile" because used in an interrupt
volatile int rightMotorSpeed = 0;
double motorSpeedVal = 0;  // how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balencing, not one for each motor
double speedVal = 0;  // how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;  // (positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;  // PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;  // PID gains for the Speed control loop

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ = 0;
int32_t rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;  // "offset" values used to zero the MPU6050 gyro on startup
uint32_t lastCalcedMPU6050 = 0;  // micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;  // rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;  // output (in degrees) from the MPU6050 reading code. negative=forwards, positive=back Pitch matters for self balancing.

hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;

rmt_config_t leftConfig;    // settings for RMT pulse for stepper motor
rmt_item32_t leftItems[1];  // holds definition of pulse for stepper motor
rmt_config_t rightConfig;
rmt_item32_t rightItems[1];

volatile boolean rightForwardBl = false;  // was the motor moving forwards last time the interrupt was called
volatile boolean leftForwardBl = false;

byte numBytesToSend = 0;
// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "sert2521";
volatile byte recvdData[maxWifiRecvBufSize] = {0};  // array to hold data recieved from DS.
volatile boolean receivedNewData = false;  // set true when data gotten, set false when parsed
volatile byte dataToSend[maxWifiSendBufSize] = {0};  // array to hold data to send to DS.
char packetBuffer[maxWifiRecvBufSize];
byte numAuxRecv = 0;  // how many bytes of control data for extra things
byte auxRecvArray[12] = {0};  // size of numAuxRecv
byte numSendAux = 0;  // how many bytes of sensor data to send
byte auxSendArray[12] = {0};  // size of numAuxSend
uint32_t lastMessageTimeMillis = 0;
byte saverecallState = 0;  // 0=don't send don't save  1=send  2=save

WiFiUDP Udp;

PID PIDA(&pitch, &motorSpeedVal, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);  // setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, DIRECT);  // setup the Speed PID loop

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(ENS_PIN, OUTPUT);
  digitalWrite(ENS_PIN, HIGH);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.begin(115200);  // for debugging. Set the serial monitor to the same value or you will see nothing or gibberish.

  mutexReceive = xSemaphoreCreateMutex();
  sprintf(robotSSID, "SERT_URSA_%02d", ROBOT_ID);  // create unique network SSID
  EEPROM.begin(64);  // size in bytes

  PIDA.SetMode(MANUAL);  // PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(1);  // tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(1);
  PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
  PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);

  recallSettings();

  setupStepperRMTs();

  setupMPU6050();  // this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();  // this function averages some gyro readings so later the readings can be calibrated to zero. This function blocks until the robot is held stil, so the robot needs to be set flat on the ground on startup

  setupWifi();

  setupStepperTimers();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {  // on core 1. the balencing control loop will be here, with the goal of keeping this loop as fast as possible
  readMPU6050();
  voltage = map(analogRead(VOLTAGE_PIN) * 1000.00 / DACUnitsPerVolt, 0, 13000.0, 0, 255);
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
      digitalWrite(ENS_PIN, LOW);  // enables stepper motors
      PIDA.SetMode(AUTOMATIC);  // turn on the PID
      PIDS.SetMode(AUTOMATIC);  // turn on the PID
    }

    PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
    PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
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
    digitalWrite(ENS_PIN, HIGH);  // disables stepper motors
  }

  wasRobotEnabled = robotEnabled;
}

byte createDataToSend() {
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

  if (saverecallState == 1) {
    recallSettings();
    addBoolToBuffer(true, counter);
    addFloatToBuffer(kP_angle, counter);
    addFloatToBuffer(kI_angle, counter);
    addFloatToBuffer(kD_angle, counter);
    addFloatToBuffer(kP_speed, counter);
    addFloatToBuffer(kI_speed, counter);
    addFloatToBuffer(kD_speed, counter);
  } else {
    addBoolToBuffer(false, counter);
  }

  return counter;
}

void parseDataReceived() {  // put parse functions here
  byte counter = 0;
  enable = readBoolFromBuffer(counter);
  speedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED * DRIVE_SPEED_SCALER, MAX_SPEED * DRIVE_SPEED_SCALER); // 0=back, 100/8=stop, 200=forwards
  turnSpeedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED * TURN_SPEED_SCALER, MAX_SPEED * TURN_SPEED_SCALER); // 0=left, 200=right
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

  saverecallState = readByteFromBuffer(counter);
  if (saverecallState == 2) {
    saveSettings();
  }
}
