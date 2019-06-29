#include "driver/rmt.h"
#include <Wire.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define ROBOT_ID 0      // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 0      // unique configuration of robot which can be used to identify additional features
#define MAX_SPEED 4000  // max speed (in steps/sec) that the motors can run at
#define MAX_TIP 33.3    // max angle in degrees the robot will attempt to recover from -- if passed, robot will disable

// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
#define LEFT_STEP_PIN GPIO_NUM_32
#define LEFT_DIR_PIN GPIO_NUM_33
#define RIGHT_STEP_PIN GPIO_NUM_25
#define RIGHT_DIR_PIN GPIO_NUM_26

// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "sert2521";

hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;
rmt_config_t configL;   // settings for RMT pulse for stepper motor
rmt_item32_t itemsL[1]; // holds definition of pulse for stepper motor
rmt_config_t configR;
rmt_item32_t itemsR[1];

WiFiServer server(80);
byte voltage = 0; //0v=0 13v=255
int signalStrength = -100;
volatile byte recvdData[50] = {0}; //array to hold data recieved from DS.
volatile boolean receivedNewData = false;//set true when data gotten, set false when parsed
volatile byte dataToSend[50] = {0}; //array to hold data to send to DS.
byte numAuxRecv = 0; //how many bytes of control data for extra things
byte auxRecvArray[12] = {0}; //size of numAuxRecv
byte numSendAux = 0; //how many bytes of sensor data to send
byte auxSendArray[12] = {0}; //size of numAuxSend

//since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexRecv;//used to check whether receiving tasks can safely change shared variables
SemaphoreHandle_t mutexSend;//used to check whether sending tasks can safely change shared variables

boolean robotEnabled = false;//enable outputs?
boolean wasRobotEnabled = false; //to know if robotEnabled has changed
boolean enable = false;//is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ, \
        rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;//"offset" values used to zero the MPU6050 gyro on startup
unsigned long lastCalcedMPU6050 = 0;//micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;//rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;//output (in degrees) from the MPU6050 reading code. matters for self balancing.

float pitchOffset = 0.000; //subtracted from the output in readMPU6050 so that zero pitch can correspond to balanced, not that the control loop cares. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not respond to perfectly balenced.
double targetPitch = 0.000;//what angle the balencing control loop should aim for the robot to be at, the output of the speed control loop
volatile int leftMotorSpeed = 0;//stepper ticks per second that the left motor is currently doing "volatile" because used in an interrupt
volatile int rightMotorSpeed = 0;
volatile boolean rightForwardBl = false;//was the motor moving forwards last time the interrupt was called
volatile boolean leftForwardBl = false;
double motorSpeedVal = 0;//how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balencing, not one for each motor
double speedVal = 0;//how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;//(positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;//PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;//PID gains for the Speed control loop
PID PIDA(&pitch, &motorSpeedVal, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);//setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, DIRECT);//setup the Speed PID loop

void IRAM_ATTR onLeftStepTimer() { //Interrupt function called by timer
  if ((leftMotorSpeed >= 0) != leftForwardBl) {//if direction has changed
    if (leftMotorSpeed >= 0) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
    } else {
      digitalWrite(LEFT_DIR_PIN, LOW);
    }
    leftForwardBl = (leftMotorSpeed >= 0);//save direction for next time
    //delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }
  rmt_write_items(configL.channel, itemsL, 1, 0);//start pulse
}
void IRAM_ATTR onRightStepTimer() { //Interrupt function called by timer
  if ((rightMotorSpeed >= 0) != rightForwardBl) {//if direction has changed
    if (rightMotorSpeed >= 0) {
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    } else {
      digitalWrite(RIGHT_DIR_PIN, LOW);
    }
    rightForwardBl = (rightMotorSpeed >= 0);//save direction for next time
    //delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }
  rmt_write_items(configR.channel, itemsR, 1, 0);//start pulse
}
void setup() {
  sprintf(robotSSID, "SERT_URSA_%02d", ROBOT_ID);  // create unique network SSID

  Serial.begin(2000000);//Set the serial monitor to the same value or you will see nothing or gibberish.
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  leftStepTimer = timerBegin(2, 80, true); // 80Mhz / 80  = 1Mhz, 1microsecond
  rightStepTimer = timerBegin(3, 80, true); // 80Mhz / 80  = 1Mhz, 1microsecond
  timerAttachInterrupt(leftStepTimer, &onLeftStepTimer, true);
  timerAttachInterrupt(rightStepTimer, &onRightStepTimer, true);
  timerAlarmWrite(leftStepTimer, 10000000000000000, true); // 1Mhz / # =  rate //practically never
  timerAlarmWrite(rightStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
  setupStepperRMTs();
  PIDA.SetMode(MANUAL);//PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(1);//tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(1);
  PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
  PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  setupMPU6050();//this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();//this function averages some gyro readings so later the readings can be calibrated to zero. This function counts on the robot being still, so the robot needs to be powered on while lying on the ground
  DBserialPrintCurrentCore("setup");//print what core this code is running on. see the declaration of this function lower down for details
  WiFi.softAP(robotSSID, robotPass);//start wifi network, code may need to be added after this to wait for it to start
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);//should never change, DS will probably need it
  server.begin();
  Serial.println("Server started");
}
void loop() {//on core 1. the balencing control loop will be here, with the goal of keeping this loop as fast as possible
  readMPU6050();
  if (abs(pitch) > MAX_TIP) {
    tipped = true;
  } else {
    tipped = false;
  }
  if (tipped) {
    robotEnabled = false;//if the robot falls over, instead of running at full speed trying to right itself as it skids along the ground, it just disables
  }
  if (receivedNewData) {
    if (xSemaphoreTake(mutexRecv, 0) == pdTRUE) {
      parseDataReceived();
      createDataToSend();
      xSemaphoreGive(mutexRecv);
    }
  }
  if (robotEnabled) {//run the following code if the robot is enabled
    if (!wasRobotEnabled) {//the robot wasn't enabled, but now it is, so this must be the first loop since it was enabled. re set up anything you might want to
      //TODO: turn on stepper motors
      PIDA.SetMode(AUTOMATIC);//turn on the PID
      PIDS.SetMode(AUTOMATIC);//turn on the PID
    }
    PIDA.SetTunings(kP_angle, kI_angle, kD_angle);
    PIDS.SetTunings(kP_speed, kI_speed, kD_speed);
    PIDS.Compute(); //compute the PID, it changes the variables you set it up with earlier.
    PIDA.Compute();
    leftMotorSpeed = constrain(motorSpeedVal + turnSpeedVal, -MAX_SPEED, MAX_SPEED); //combine motor speed and turn to find the speed the left motor should go
    rightMotorSpeed = constrain(motorSpeedVal - turnSpeedVal, -MAX_SPEED, MAX_SPEED); //combine motor speed and turn to find the speed the right motor should go
    if (abs(leftMotorSpeed) >= 1) {
      timerAlarmWrite(leftStepTimer, 1000000 / leftMotorSpeed, true); // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(leftStepTimer, 10000000000000000, true); //don't step
    }
    if (abs(rightMotorSpeed) >= 1) {
      timerAlarmWrite(rightStepTimer, 1000000 / rightMotorSpeed, true); // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(rightStepTimer, 10000000000000000, true); // don't step
    }
  } else {//disable
    PIDA.SetMode(MANUAL);
    PIDS.SetMode(MANUAL);
    timerAlarmWrite(leftStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
    timerAlarmWrite(rightStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    //TODO: turn off stepper motors
    delay(2000);
    DBserialPrintCurrentCore("(disabled) loop");//print debugging test of what core the loop is running on
  }
  wasRobotEnabled = robotEnabled;
}
void createDataToSend() {//put send functions here
  byte counter = 0;
  sendBufferAddBoolean(robotEnabled, counter);
  sendBufferAddBoolean(tipped, counter);
  sendBufferAddByte(ROBOT_ID, counter);
  sendBufferAddByte(MODEL_NO, counter);
  sendBufferAddFloat(pitch, counter);
  sendBufferAddByte(voltage, counter);
  sendBufferAddByte(constrain(map(signalStrength, -180, 10, 0, 255), 0, 255), counter); //wifi RSSI higher=better TODO: adjust range
  sendBufferAddInt(leftMotorSpeed, counter);
  sendBufferAddInt(rightMotorSpeed, counter);
  sendBufferAddByte(numSendAux, counter);//how many bytes of extra data
  for (int i = 0; i < numSendAux; i++) {
    sendBufferAddByte(auxSendArray[numSendAux], counter); //extra data
  }
}
void parseDataReceived() {//put parse functions here
  byte counter = 0;
  enable = receiveBufferReadBoolean(counter);
  speedVal = map(receiveBufferReadByte(counter), 0, 255, -MAX_SPEED, MAX_SPEED); //0=back, 127/8=stop, 255=forwards
  turnSpeedVal = map(receiveBufferReadByte(counter), 0, 255, -MAX_SPEED / 50, MAX_SPEED / 50); //0=left, 255=right
  numAuxRecv = receiveBufferReadByte(counter); //how many bytes of control data for extra things
  for (int i = 0; i < numAuxRecv; i++) {
    auxRecvArray[i] = receiveBufferReadByte(counter);
  }
  if (receiveBufferReadBoolean(counter)) {
    kP_angle = receiveBufferFloat(counter);
    kI_angle = receiveBufferFloat(counter);
    kD_angle = receiveBufferFloat(counter);
    kP_speed = receiveBufferFloat(counter);
    kI_speed = receiveBufferFloat(counter);
    kD_speed = receiveBufferFloat(counter);
  }
}
void setupStepperRMTs() {
  configL.rmt_mode = RMT_MODE_TX;
  configL.channel = RMT_CHANNEL_0;
  configL.gpio_num = LEFT_STEP_PIN;
  configL.mem_block_num = 1;
  configL.tx_config.loop_en = 0;
  configL.tx_config.carrier_en = 0;
  configL.tx_config.idle_output_en = 1;
  configL.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  configL.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  configL.clk_div = 80; // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&configL);
  rmt_driver_install(configL.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  itemsL[0].duration0 = 2;
  itemsL[0].level0 = 1;
  itemsL[0].duration1 = 0;
  itemsL[0].level1 = 0;

  configR.rmt_mode = RMT_MODE_TX;
  configR.channel = RMT_CHANNEL_0;
  configR.gpio_num = RIGHT_STEP_PIN;
  configR.mem_block_num = 1;
  configR.tx_config.loop_en = 0;
  configR.tx_config.carrier_en = 0;
  configR.tx_config.idle_output_en = 1;
  configR.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  configR.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  configR.clk_div = 80; // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&configR);
  rmt_driver_install(configR.channel, 0, 1);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  itemsR[0].duration0 = 2;
  itemsR[0].level0 = 1;
  itemsR[0].duration1 = 0;
  itemsR[0].level1 = 0;
}
void DBserialPrintCurrentCore(String msg) { //function for DeBugging, packages and prints the core the function is called from
  Serial.print(msg);
  Serial.print(" running on core #");
  Serial.println(xPortGetCoreID());//prints which core this code is running on
}
void WiFiEvent(WiFiEvent_t event) {//this function is hopefully called automatically when something wifiy happens, including recieving a message
  DBserialPrintCurrentCore("wifi event");
  Serial.print("wifi event: ");
  Serial.println(event);
  Serial.print("WiFi.RSSI=");
  Serial.println(WiFi.RSSI());//Recieved Signal Strength Indicator, less negative numbers mean a stronger recieved signal
  //wifi recieve code:
  WiFiClient client = server.available();
  if (client) {
    if (xSemaphoreTake(mutexRecv, 0) == pdTRUE) {
      Serial.println("got wifi recieve mutex");
      byte count = 0;
      while (client.available()) {
        char c = client.read();
        recvdData[count] = c;
        count++;
        receivedNewData = true;
        Serial.write(c);
      }
      for (int i = 0; i < count; i++) {//send response, maybe change to go less frequently
        client.write(dataToSend[i]);
      }
      xSemaphoreGive(mutexRecv);
    }
    Serial.println(" wifi end");
  }
}
//start I2C communication and send commands to set up the MPU6050.
//A command is set by starting a transmission, writing a byte (written here in hexadecimal) to signal what register should be changed,
//and then sending a new register value
void setupMPU6050() {
  Wire.begin();//////////////////////setup mpu6050. reference for the mpu6050's registers https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  Wire.setClock(400000L);//send data at a faster clock speed
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);//wakeup
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);//gyro range //18=2000//10=1000
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x19);
  Wire.write(0x02);//clock divider
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x00);//buffering
  Wire.endTransmission(true);///////end setup mpu6050
}
void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);//location of first byte of data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);//ask for accel and gyro data bytes
  accelerationX = Wire.read() << 8 | Wire.read();//read two bytes and put them together into a sixteen bit integer value
  accelerationY = Wire.read() << 8 | Wire.read();
  accelerationZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();//throw away temperature, it's annoying they put it in the middle here
  rotationX = Wire.read() << 8 | Wire.read();
  rotationY = Wire.read() << 8 | Wire.read();
  rotationZ = Wire.read() << 8 | Wire.read();
  rotationDPS_X = (rotationX - rotationOffsetX) * 1000000.00 / 32766;//zero gyro with offset values recorded on startup and convert to degrees per second
  rotationDPS_Y = (rotationY - rotationOffsetY) * 1000000.00 / 32766;
  rotationDPS_Z = (rotationZ - rotationOffsetZ) * 1000000.00 / 32766;
  if (micros() > lastCalcedMPU6050) {//try to handle micros' long overflow in a harmless way
    lastCalcedMPU6050 = micros() - 10;
  }
  pitch = .99 * ((pitch - pitchOffset) + rotationDPS_X * (micros() - lastCalcedMPU6050) / 1000000.000) + .01 * (degrees(atan2(accelerationY, accelerationZ)) - pitchOffset); //complementary filter combines gyro and accelerometer tilt data in a way that takes advantage of short term accuracy of the gyro and long term accuracy of the accelerometer
  lastCalcedMPU6050 = micros();//record time of last calculation so we know next time how much time has passed (how much time to integrate rotation rate for)
}
void zeroMPU6050() {//find how much offset each gyro axis has to zero out drift. should be run on startup (when robot is still)
  rotationOffsetX = 0;
  rotationOffsetY = 0;
  rotationOffsetZ = 0;
  for (int i = 0; i < 50; i++) {//run the following code 50 times so we can get many measurements to average into an offset value
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    accelerationX = Wire.read() << 8 | Wire.read();//same reading code as earlier
    accelerationY = Wire.read() << 8 | Wire.read();
    accelerationZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();//throw away temperature
    rotationX = Wire.read() << 8 | Wire.read();
    rotationY = Wire.read() << 8 | Wire.read();
    rotationZ = Wire.read() << 8 | Wire.read();
    rotationOffsetX += rotationX;//add all the reads together
    rotationOffsetY += rotationY;
    rotationOffsetZ += rotationZ;
    delay(10 + i / 5);//add some time between reads, changing the delay each time a bit to be less likely to be thrown by a periodic oscillation
  }
  rotationOffsetX /= 50;//devide by the number of reads that were taken to get an average value
  rotationOffsetY /= 50;
  rotationOffsetZ /= 50;
}
boolean receiveBufferReadBoolean(byte &pos) {//return boolean at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;//increment the counter for the next value
  return (msg == 1);
}
byte receiveBufferReadByte(byte &pos) {//return byte at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;//increment the counter for the next value
  return msg;
}
int receiveBufferReadInt(byte &pos) { //return int from two bytes starting at pos position in recvdData
  union {//this lets us translate between two variable types (equal size, but one's two bytes in an array, and one's a two byte int)  Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[2];
    int v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the int
  d.b[0] = recvdData[pos]; //read the first byte
  pos++;//increment i to the location of the second byte
  d.b[1] = recvdData[pos]; //read the second byte
  pos++;//shift i once more so it's ready for the next function (at the position of the start of the next value)
  return d.v;//return the int form of union d
}
float receiveBufferFloat(byte &pos) {//return float from 4 bytes starting at pos position in recvdData
  union {//this lets us translate between two variable types (equal size, but one's 4 bytes in an array, and one's a 4 byte float) Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the float
  d.b[0] = recvdData[pos];
  pos++;
  d.b[1] = recvdData[pos];
  pos++;
  d.b[2] = recvdData[pos];
  pos++;
  d.b[3] = recvdData[pos];
  pos++;
  return d.v;
}
void sendBufferAddBoolean(boolean msg, byte &pos) {//add a boolean to the tosendData array
  dataToSend[pos] = msg;
  pos++;
}
void sendBufferAddByte(byte msg, byte &pos) {//add a byte to the tosendData array
  dataToSend[pos] = msg;
  pos++;
}
void sendBufferAddInt(int msg, byte &pos) {//add an int to the tosendData array (two bytes)
  union {
    byte b[2];
    int v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the int
  d.v = msg;//put the value into the union as an int
  dataToSend[pos] = d.b[0];
  pos++;
  dataToSend[pos] = d.b[1];
  pos++;
}
void sendBufferAddFloat(float msg, byte &pos) {//add a float to the tosendData array (four bytes)
  union {//this lets us translate between two variables (equal size, but one's 4 bytes in an array, and one's a 4 byte float Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the float
  d.v = msg;
  dataToSend[pos] = d.b[0];
  pos++;
  dataToSend[pos] = d.b[1];
  pos++;
  dataToSend[pos] = d.b[2];
  pos++;
  dataToSend[pos] = d.b[3];
  pos++;
}
