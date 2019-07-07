#include "driver/rmt.h"
#include <Wire.h>  // scl=22 sda=21
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define ROBOT_ID 0      // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 0      // unique configuration of robot which can be used to identify additional features
#define WiFiLossDisableIntervalMillis 1000    // if no data packet has been recieved for this number of milliseconds, the robot disables to prevent running away
float COMPLEMENTARY_FILTER_CONSTANT = .9997;  // higher = more gyro based, lower=more accelerometer based
int MAX_SPEED = 4000;  // max speed (in steps/sec) that the motors can run at
float MAX_TIP = 60;  // max angle in degrees the robot will attempt to recover from -- if passed, robot will disable

// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
#define LEFT_STEP_PIN GPIO_NUM_32
#define LEFT_DIR_PIN GPIO_NUM_33
#define RIGHT_STEP_PIN GPIO_NUM_34
#define RIGHT_DIR_PIN GPIO_NUM_35
#define ENS_PIN GPIO_NUM_23  // pin wired to both motor driver chips' ENable pins, to turn on and off motors
#define LED_BUILTIN 2

#define movementThreshold 34
#define movementMeasurements 15

#define maxWifiRecvBufSize 50  // max number of bytes to receive
#define maxWifiSendBufSize 50  // max number of bytes to send
byte numBytesToSend = 0;

// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "sert2521";
hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;

rmt_config_t leftConfig;    // settings for RMT pulse for stepper motor
rmt_item32_t leftItems[1];  // holds definition of pulse for stepper motor
rmt_config_t rightConfig;
rmt_item32_t rightItems[1];

WiFiUDP Udp;

byte voltage = 0;  // 0v=0 13v=255
volatile byte recvdData[maxWifiRecvBufSize] = {0};  // array to hold data recieved from DS.
volatile boolean receivedNewData = false;  // set true when data gotten, set false when parsed
volatile byte dataToSend[maxWifiSendBufSize] = {0};  // array to hold data to send to DS.
byte numAuxRecv = 0;  // how many bytes of control data for extra things
byte auxRecvArray[12] = {0};  // size of numAuxRecv
byte numSendAux = 0;  // how many bytes of sensor data to send
byte auxSendArray[12] = {0};  // size of numAuxSend
unsigned long lastMessageTimeMillis = 0;

// since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexReceive;  // used to check whether receiving tasks can safely change shared variables

boolean robotEnabled = false;     // enable outputs?
boolean wasRobotEnabled = false;  // to know if robotEnabled has changed
boolean enable = false;           // is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ = 0;
int32_t rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;  // "offset" values used to zero the MPU6050 gyro on startup
unsigned long lastCalcedMPU6050 = 0;  // micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;  // rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;  // output (in degrees) from the MPU6050 reading code. negative=forwards, positive=back Pitch matters for self balancing.

float pitchOffset = 0.000;  // subtracted from the output in readMPU6050 so that zero pitch can correspond to balanced, not that the control loop cares. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not respond to perfectly balenced.
double targetPitch = 0.000;  // what angle the balencing control loop should aim for the robot to be at, the output of the speed control loop
volatile int leftMotorSpeed = 0;  // stepper ticks per second that the left motor is currently doing "volatile" because used in an interrupt
volatile int rightMotorSpeed = 0;
volatile boolean rightForwardBl = false;  // was the motor moving forwards last time the interrupt was called
volatile boolean leftForwardBl = false;
double motorSpeedVal = 0;  // how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balencing, not one for each motor
double speedVal = 0;  // how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;  // (positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;  // PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;  // PID gains for the Speed control loop
PID PIDA(&pitch, &motorSpeedVal, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);  // setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, DIRECT);  // setup the Speed PID loop

void IRAM_ATTR onLeftStepTimer() {  // Interrupt function called by timer
  if ((leftMotorSpeed >= 0) != leftForwardBl) {  // if direction has changed
    if (leftMotorSpeed >= 0) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
    } else {
      digitalWrite(LEFT_DIR_PIN, LOW);
    }

    leftForwardBl = (leftMotorSpeed >= 0);  // save direction for next time

    // delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }

  rmt_write_items(leftConfig.channel, leftItems, 1, 0);  // start pulse
}

void IRAM_ATTR onRightStepTimer() {  // Interrupt function called by timer
  if ((rightMotorSpeed >= 0) != rightForwardBl) {  // if direction has changed
    if (rightMotorSpeed >= 0) {
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    } else {
      digitalWrite(RIGHT_DIR_PIN, LOW);
    }

    rightForwardBl = (rightMotorSpeed >= 0);  // save direction for next time

    // delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }

  rmt_write_items(rightConfig.channel, rightItems, 1, 0);  // start pulse
}

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
  setupStepperRMTs();

  PIDA.SetMode(MANUAL);  // PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(1);  // tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(1);
  PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
  PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);

  setupMPU6050();  // this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();  // this function averages some gyro readings so later the readings can be calibrated to zero. This function counts on the robot being still, so the robot needs to be powered on while lying on the ground

  WiFi.softAPConfig(IPAddress(10, 25, 21, 1), IPAddress(10, 25, 21, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(robotSSID, robotPass);  // start wifi network, code may need to be added after this to wait for it to start
  IPAddress myIP = WiFi.softAPIP();
  Udp.begin(2521);  // port 2521 on 10.25.21.1 -needed by DS

  xTaskCreatePinnedToCore(  // create task to run WiFi recieving
    WiFiTaskFunction,   /* Function to implement the task */
    "WiFiTask",  /* Name of the task */
    10000,       /* Stack size in words */
    NULL,        /* Task input parameter */
    1,           /* Priority of the task */
    NULL,        /* Task handle. */
    0            /* Core on which task should run */
  );

  leftStepTimer = timerBegin(2, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  rightStepTimer = timerBegin(3, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  timerAttachInterrupt(leftStepTimer, &onLeftStepTimer, true);
  timerAttachInterrupt(rightStepTimer, &onRightStepTimer, true);
  timerAlarmWrite(leftStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate // practically never
  timerAlarmWrite(rightStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate

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

int createDataToSend() {
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

void setupStepperRMTs() {
  leftConfig.rmt_mode = RMT_MODE_TX;
  leftConfig.channel = RMT_CHANNEL_0;
  leftConfig.gpio_num = LEFT_STEP_PIN;
  leftConfig.mem_block_num = 1;
  leftConfig.tx_config.loop_en = 0;
  leftConfig.tx_config.carrier_en = 0;
  leftConfig.tx_config.idle_output_en = 1;
  leftConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  leftConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  leftConfig.clk_div = 80;  // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&leftConfig);
  rmt_driver_install(leftConfig.channel, 0, 0);  // rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  leftItems[0].duration0 = 2;
  leftItems[0].level0 = 1;
  leftItems[0].duration1 = 0;
  leftItems[0].level1 = 0;

  rightConfig.rmt_mode = RMT_MODE_TX;
  rightConfig.channel = RMT_CHANNEL_1;
  rightConfig.gpio_num = RIGHT_STEP_PIN;
  rightConfig.mem_block_num = 1;
  rightConfig.tx_config.loop_en = 0;
  rightConfig.tx_config.carrier_en = 0;
  rightConfig.tx_config.idle_output_en = 1;
  rightConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rightConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  rightConfig.clk_div = 80;  // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&rightConfig);
  rmt_driver_install(rightConfig.channel, 0, 1);  // rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  rightItems[0].duration0 = 2;
  rightItems[0].level0 = 1;
  rightItems[0].duration1 = 0;
  rightItems[0].level1 = 0;
}
