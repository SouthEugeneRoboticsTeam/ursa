/*
Motor Sensor Wiring
  0 Red
  2 Black
  4 White (Motor 1)
  12 Yellow
  13 White (Motor 2)
  GND Purple
  +3.3V Green


IMU Wiring
  +3.3V IMU VCC
  GND IMU Ground
  16(SDA) IMU SDA
  17(SCL) IMU SCL
*/


#include <dummy.h>

#include <PID_v1.h>
#include <Wire.h>  // scl=22 sda=21
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <SimpleFOC.h>

#define ROBOT_ID 4  // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 3  // unique configuration of robot which can be used to identify additional features
#define WiFiLossDisableIntervalMillis 500  // if no data packet has been recieved for this number of milliseconds, the robot disables to prevent running away
// #define DACUnitsPerVolt 200.0  // Use to calibrate voltage read through voltage divider. Divide analogRead value by this constant to get voltage. Analog read is from 0 to 4095 corresponding to 0 to 3.3 volts.
float MAX_ACCEL = 2;  // limits maximum change in speed value per loop (originally 180, then 100)
float COMPLEMENTARY_FILTER_CONSTANT = .9997;  // higher = more gyro based, lower=more accelerometer based
//int MAX_SPEED = 1500;  // max speed (in steps/sec) that the motors can run at
float MAX_SPEED = 30.0;  // max speed (in steps/sec) that the motors can run at (originally 1500)
float MAX_TIP = 20;  // angle the robot shouldn't go too much past, the output limit for the speed PID loop
float DISABLE_TIP = 30; // max angle in degrees the robot will attempt to recover from -- if passed, robot will disable
float DRIVE_SPEED_SCALER = .85;  // what proportion of MAX_SPEED the robot's target driving speed can be-some extra speed must be kept in reserve to remain balanced
float TURN_SPEED_SCALER = .15;  // what proportion of MAX_SPEED can be given differently to each wheel in order to turn-controls maximum turn rate
float PITCH_OFFSET_CHANGE = .999994;  // larger = pitchOffset changes slower
float pitchOffset = 0.00;  // subtracted from the output in readMPU6050 so that zero pitch can correspond to balenced. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not otherwise respond to perfectly balanced.

// Set up the rgb led names; used to control RGB LEDs
/*uint8_t RightledR = A15; //GPIO12
uint8_t RightledG = A16; //GPIO14
uint8_t RightledB = A17; //GPIO27
uint8_t LeftledR = A10; //GPIO4
uint8_t LeftledG = A11; //GPIO0
uint8_t LeftledB = A12; //GPIO2
*/
int RED = 256;
int GREEN = 256;
int BLUE = 256;

//**************************************************These should all be eliminated once gimbals are running
// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
//#define LEFT_STEP_PIN GPIO_NUM_32
//#define LEFT_DIR_PIN GPIO_NUM_33
//#define RIGHT_STEP_PIN GPIO_NUM_34 // was 24, changed to 34 so that it didn't overlap with the gimbal sensor GPIO
//#define RIGHT_DIR_PIN GPIO_NUM_35 // was 26, changed to 35 so that it didn't overlap with the gimbal sensor GPIO
//#define ENS_PIN GPIO_NUM_39  // pin wired to both motor driver chips' ENable pins, to turn on and off motors    // was 23, changed to 39 so that it didn't overlap with the gimbal sensor GPIO
//***********************************************************************

// Set up alternative SPI pinout
#define HSPI_MISO 19
#define HSPI_MOSI 26
#define HSPI_SCLK 3
#define HSPI_SS 17
#define HSPI_SS2 18
SPIClass SPI_2(HSPI);

// Define gimbal sensors, drivers and motors
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, HSPI_SS);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, HSPI_SS2);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(16, 13, 27, 12);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(25, 5, 23, 14);

#define LED_BUILTIN GPIO_NUM_2
#define VOLTAGE_PIN GPIO_NUM_36  // ADC1 CH0

#define movementThreshold 20
#define movementMeasurements 20

#define maxWifiRecvBufSize 50  // max number of bytes to receive
#define maxWifiSendBufSize 51  // max number of bytes to send

byte voltage_test = 0;  // 0v=0 13v=255

// since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexReceive;  // used to check whether receiving tasks can safely change shared variables

boolean robotEnabled = false;     // enable outputs?
boolean wasRobotEnabled = false;  // to know if robotEnabled has changed
boolean enable = false;           // is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;
boolean kickstart = false;

double targetPitch = 0.000;  // what angle the balancing control loop should aim for the robot to be at, the output of the speed control loop
double motorSpeed = 0.000;  // how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balancing, not one for each motor
//volatile int leftMotorWriteSpeed = 0;  // after acceleration
//volatile int rightMotorWriteSpeed = 0;
double leftMotorWriteSpeed = 0.0;  // after acceleration
double rightMotorWriteSpeed = 0.0;
double motorAccel = 0.0;  // how many stepper ticks per second per loop cycle the motors should be made to accelerate at, used as output of angle balancing loop
double speedVal = 0.0;  // how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
double turnSpeedVal = 0.0;  // (positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;  // PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;  // PID gains for the Speed control loop

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ = 0;
int32_t rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;  // "offset" values used to zero the MPU6050 gyro on startup
uint32_t lastCalcedMPU6050 = 0;  // micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;  // rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;  // output (in degrees) from the MPU6050 reading code. negative=forwards, positive=back Pitch matters for self balancing.
double pitch1 = 0.000;
double pitch2 = 0.000;
double pitch3 = 0.000;
double pitchInverter = -1.0;

hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;

byte numBytesToSend = 0;
// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "Headbot2521";
volatile byte recvdData[maxWifiRecvBufSize] = {0};  // array to hold data recieved from DS.
volatile boolean receivedNewData = false;  // set true when data gotten, set false when parsed
volatile byte dataToSend[maxWifiSendBufSize] = {0};  // array to hold data to send to DS.
char packetBuffer[maxWifiRecvBufSize];
byte numAuxRecv = 0;  // how many bytes of control data for extra things
byte auxRecvArray[12] = {0};  // size of numAuxRecv
byte numSendAux = 0;  // how many bytes of sensor data to send
byte auxSendArray[12] = {0};  // size of numAuxSend
volatile uint32_t lastMessageTimeMillis = 0;
byte saverecallState = 0;  // 0=don't send don't save  1=send  2=save

WiFiUDP Udp;

PID PIDA(&pitch, &motorAccel, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);  // setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
//PID PIDS(&motorSpeed, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, REVERSE);  // setup the Speed PID loop
PID PIDS(&motorSpeed, &targetPitch, &speedVal, kP_speed, kI_speed, kD_speed, REVERSE);  // setup the Speed PID loop

void setup() {
  /*pinMode(LED_BUILTIN, OUTPUT);
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
*/
  Serial.begin(115200);  // for debugging. Set the serial monitor to the same value or you will see nothing or gibberish.

  mutexReceive = xSemaphoreCreateMutex();
  sprintf(robotSSID, "SERT_URSA_%02d", ROBOT_ID);  // create unique network SSID
  EEPROM.begin(64);  // size in bytes

  PIDA.SetMode(MANUAL);  // PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(10);  // tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(10);
  PIDA.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
  PIDS.SetOutputLimits(-MAX_TIP, MAX_TIP);

  // used to control RGB LEDs
  /*ledcAttachPin(RightledR, 1); // assign RGB led pins to channels
  ledcAttachPin(RightledG, 2);
  ledcAttachPin(RightledB, 3);
  ledcAttachPin(LeftledR, 4); // assign RGB led pins to channels
  ledcAttachPin(LeftledG, 5);
  ledcAttachPin(LeftledB, 6);
  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 5000, 8); // 5 kHz PWM, 8-bit resolution
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  ledcSetup(4, 5000, 8);
  ledcSetup(5, 5000, 8);
  ledcSetup(6, 5000, 8);
*/

  recallSettings();

  setupMPU6050();  // this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();  // this function averages some gyro readings so later the readings can be calibrated to zero. This function blocks until the robot is held stil, so the robot needs to be set flat on the ground on startup

  setupWifi();

  setupStepperTimers(); //******************************************************** Is this needed with the gimbal motors? My guess is no

  // added for gimbal motors
  // start SPI communication for gimbal sensors
  SPI_2.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS 
  // initialise magnetic sensor hardware
  sensor1.init(&SPI_2);
  sensor2.init(&SPI_2);
  Serial.println("Sensor ready");
  _delay(1000);
  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  // driver config
  driver1.voltage_power_supply = 11.1;   // power supply voltage [V]
  driver2.voltage_power_supply = 11.1;   // power supply voltage [V]
  driver1.init();
  driver2.init();
  // link the motor and the driver
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  // set motion control loop to be used
  motor1.controller = MotionControlType::velocity;
  motor2.controller = MotionControlType::velocity;
  // contoller configuration; default parameters in defaults.h (of SimpleFOC.h, I think?)
  // velocity PI controller parameters
  motor1.PID_velocity.P = 0.2f;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0;
  // default voltage_power_supply
  motor1.voltage_limit = 11.1;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor1.PID_velocity.output_ramp = 1000;
  // velocity PI controller parameters
  motor2.PID_velocity.P = 0.2f;
  motor2.PID_velocity.I = 20;
  motor2.PID_velocity.D = 0;
  // default voltage_power_supply
  motor2.voltage_limit = 11.1;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor2.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01f;
  motor2.LPF_velocity.Tf = 0.01f;
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  // initialize motor
  motor1.init();
  motor2.init();
  // align sensor and start FOC
  motor1.initFOC();
  motor2.initFOC();
  Serial.println(F("Motor ready."));
  _delay(1000);


  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {  // on core 1. the balancing control loop will be here, with the goal of keeping this loop as fast as possible

  readMPU6050();

  // voltage = map(analogRead(VOLTAGE_PIN) * 1000.00 / DACUnitsPerVolt, 0, 13000.0, 0, 255);
  voltage_test = map(analogRead(VOLTAGE_PIN), 0, 4095.0, 0, 255);

  if (receivedNewData) {
    if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
      parseDataReceived();
      numBytesToSend = createDataToSend();
      receivedNewData = false;
      xSemaphoreGive(mutexReceive);
      GREEN = 256;
    }
  }

  robotEnabled = enable;

  if (abs(pitch) >= DISABLE_TIP && !kickstart) {
//  if (abs(pitch) >= DISABLE_TIP) {
    tipped = true;
    robotEnabled = false;
    BLUE = 0;
  } else {
    tipped = false;
    BLUE = 256;
  }

  if (abs(pitch) < (DISABLE_TIP - 3)) {
    tipped = false;
    kickstart = false;
  }

  if (millis() - lastMessageTimeMillis > WiFiLossDisableIntervalMillis) {
   // robotEnabled = false;
    speedVal = 0;
    turnSpeedVal = 0;
    GREEN = 0;
  }

  if (robotEnabled) {  // run the following code if the robot is enabled
 
   /* ledcWrite(1, (abs(256 - int(millis() % 5120) / 10))*.9);    //causes LEDs to throb red
    //ledcWrite(2, 128+(abs(256 - int(millis() % 5120) / 10))*.9*.5);
    ledcWrite(2, 256);
    //ledcWrite(3, 192+(abs(256 - int(millis() % 5120) / 10))*.9*.25);
    ledcWrite(3, (abs(256 - int(millis() % 5120) / 10))*.9);
    ledcWrite(4, (abs(256 - int(millis() % 5120) / 10))*.9);    //causes LEDs to throb red
    //ledcWrite(5, 256);
    ledcWrite(5, 256);
    //ledcWrite(6, 192+(abs(256 - int(millis() % 5120) / 10))*.9*.25);
    ledcWrite(6, (abs(256 - int(millis() % 5120) / 10))*.9);
    //ledcWrite(1, RED);
    //ledcWrite(2, GREEN);
    //ledcWrite(3, BLUE);
    //ledcWrite(4, RED);
    //ledcWrite(5, GREEN);
    //ledcWrite(6, BLUE);
    
    //digitalWrite(LED_BUILTIN, (millis() % 500 < 250));
 */
    if (!wasRobotEnabled) {  // the robot wasn't enabled, but now it is, so this must be the first loop since it was enabled. re set up anything you might want to
 //     digitalWrite(ENS_PIN, LOW);  // enables stepper motors
      PIDA.SetMode(AUTOMATIC);  // turn on the PID
      PIDS.SetMode(AUTOMATIC);  // turn on the PID
      motor1.enable();
      motor2.enable();
    }

    PIDA.SetTunings(kP_angle, kI_angle, kD_angle);
    PIDS.SetTunings(kP_speed, kI_speed, kD_speed);
    PIDS.Compute();  // compute the PID, it changes the variable (motorSpeedVal) you set it up with earlier.
    PIDA.Compute();
  
    motorSpeed += constrain(motorAccel, -MAX_ACCEL, MAX_ACCEL);
    motorSpeed = constrain(motorSpeed, -MAX_SPEED, MAX_SPEED);
    leftMotorWriteSpeed = constrain(motorSpeed - turnSpeedVal, -MAX_SPEED, MAX_SPEED); // combine turnSpeedVal and the motor speed required for forwards/backwards movement so the robot can move and turn
    rightMotorWriteSpeed = constrain(motorSpeed + turnSpeedVal, -MAX_SPEED, MAX_SPEED); // positive turn=turn to the right -> right wheel needs to slow down -> subtract turnSpeedVal for right motor
/*
    if (leftMotorWriteSpeed >= 0) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
    } else {
      digitalWrite(LEFT_DIR_PIN, LOW);
    }
    if (rightMotorWriteSpeed >= 0) {
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    } else {
      digitalWrite(RIGHT_DIR_PIN, LOW);
    }

    if (abs(leftMotorWriteSpeed) >= 1) {
     timerAlarmWrite(leftStepTimer, 1000000 / abs(leftMotorWriteSpeed), true);  // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(leftStepTimer, 1e17, true);  // don't step
    }
    if (abs(rightMotorWriteSpeed) >= 1) {
      timerAlarmWrite(rightStepTimer, 1000000 / abs(rightMotorWriteSpeed), true);  // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(rightStepTimer, 1e17, true);  // don't step
    }
*/
 //   timerAlarmEnable(leftStepTimer);
 //   timerAlarmEnable(rightStepTimer);


    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor1.loopFOC();
    motor2.loopFOC();

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    //motor.move(target_velocity);
    //Serial.println(leftMotorWriteSpeed);
    motor1.move(leftMotorWriteSpeed);
    motor2.move(rightMotorWriteSpeed);


  } else {  // disable
    //ledcWrite(1, RED); //turns LEDs off
    //ledcWrite(2, GREEN);
    //ledcWrite(3, BLUE);
    //ledcWrite(4, RED); //turns LEDs off
    //ledcWrite(5, GREEN);
    //ledcWrite(6, BLUE);
/*
    ledcWrite(1, (abs(256 - int(millis() % 5120) / 10))*.9);    //causes LEDs to throb red
    ledcWrite(2, 256);
    ledcWrite(3, 256);
    ledcWrite(4, (abs(256 - int(millis() % 5120) / 10))*.9);    //causes LEDs to throb red
    ledcWrite(5, 256);
    ledcWrite(6, 256);
*/


    //digitalWrite(LED_BUILTIN, HIGH);
    PIDA.SetMode(MANUAL);
    PIDS.SetMode(MANUAL);
//    timerAlarmWrite(leftStepTimer, 1e17, true);  // 1Mhz / # =  rate
//    timerAlarmWrite(rightStepTimer, 1e17, true);  // 1Mhz / # =  rate
//    timerAlarmEnable(leftStepTimer);
//    timerAlarmEnable(rightStepTimer);
    leftMotorWriteSpeed = 0;
    rightMotorWriteSpeed = 0;
    targetPitch = 0;
    motorAccel = 0;
    motorSpeed = 0;
//    digitalWrite(ENS_PIN, HIGH);  // disables stepper motors
    motor1.move(leftMotorWriteSpeed);
    motor2.move(rightMotorWriteSpeed);
    motor1.disable();
    motor2.disable();
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
  addByteToBuffer(voltage_test, counter);
  addIntToBuffer(leftMotorWriteSpeed, counter);
  addIntToBuffer(rightMotorWriteSpeed, counter);
  addFloatToBuffer(targetPitch, counter);
  addFloatToBuffer(pitchOffset, counter);
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
  speedVal = map(readByteFromBuffer(counter), 200, 0, -MAX_SPEED * DRIVE_SPEED_SCALER, MAX_SPEED * DRIVE_SPEED_SCALER);
  turnSpeedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED * TURN_SPEED_SCALER, MAX_SPEED * TURN_SPEED_SCALER);
  numAuxRecv = readByteFromBuffer(counter);  // how many bytes of control data for extra things

  if (enable && !robotEnabled) {
    kickstart = true;
  }

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
