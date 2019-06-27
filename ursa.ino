#define MAX_TIP 33.3 //max angle the robot might recover from, if this angle is passed,the robot disables to not keep skidding along the ground
#define ID 0//robot ID, sent to DS, could be used to identify what unique robot this is, and what attachments it has
#define MAX_SPEED 4000 //maximum steps per second that the motors can do
#define leftStepPin GPIO_NUM_32//esp pin connected to the left stepper driver's STEP pin. RISING low to high triggers step
#define leftDirPin GPIO_NUM_33//esp pin connected to the left stepper driver's DIR pin. changes which direction the motor is driven
#define rightStepPin GPIO_NUM_25//esp pin connected to the right stepper driver's STEP pin. RISING low to high triggers step
#define rightDirPin GPIO_NUM_26//esp pin connected to the right stepper driver's DIR pin. changes which direction the motor is driven
const char *robotSSID = "SERT_URSA_0";//name of robot's wifi hotspot, should be unique between all robots
const char *robotPass = "sert2521";//password for the robot's wifi network, not very secure but it might discourage random people from connecting and messing up our communication
#include "driver/rmt.h"
#include <Wire.h>//arduino library used for I2C communication with the mpu6050 gyro board Reference for Wire: https://www.arduino.cc/en/Reference/Wire
#include <PID_v1.h>//arduino library for PID loop, we could write our own, but this library is packaged nicely Library: https://github.com/br3ttb/Arduino-PID-Library Reference: https://playground.arduino.cc/Code/PIDLibrary/
#include <WiFi.h>//esp32 wifi library
#include <WiFiClient.h>//esp32 wifi library
#include <WiFiAP.h>//esp32 wifi library for creating wifi network
hw_timer_t * leftStepTimer = NULL; //here's a timer to use for timing motor steps Reference: https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino
hw_timer_t * rightStepTimer = NULL; //here's a timer to use for timing motor steps Reference: https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino
rmt_config_t configL;
rmt_item32_t itemsL[1];
rmt_config_t configR;
rmt_item32_t itemsR[1];
boolean robotEnabled = false;//is the robot enabled? if false, turn off all outputs
boolean wasRobotEnabled = false; //keep track of whether the robot was enabled last loop to know if it has changed.
boolean enable = false;//is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;//has the robot tipped past it's maximum recoverable angle?
int16_t oAX, oAY, oAZ, oRX, oRY, oRZ, oRX0, oRY0, oRZ0 = 0;//for MPU6050 A=acceleration(raw) R=rotation(raw) R_0=values used to zero the gyro on startup
unsigned long lastCalcedMPU6050 = 0;//micros() value of last orientation read. used to integrate gyro data to get rotation
double oDPSX, oDPSY, oDPSZ = 0.000;//rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;//angle of pitch of the robot forward and backwards in degrees. The output from the MPU6050 that matters for self balencing.
float pitchOffset = 0.000; //subtracted from the output in readMPU6050 so that zero pitch can correspond to balanced, not that the control loop cares. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not respond to perfectly balenced.
volatile int leftMotorSpeed = 0;//stepper ticks per second that the left motor is currently doing "volatile" because used in an interrupt
volatile int rightMotorSpeed = 0;//stepper ticks per second that the right motor is currently doing "volatile" because used in an interrupt
volatile boolean rightForwardBl = false;//was the motor moving forwards last time the interrupt was called
volatile boolean leftForwardBl = false;//was the motor moving forwards last time the interrupt was called
double motorSpeedVal = 0;//average stepper ticks per second that the two motors should do-how much movement in the forwards/backwards direction should they move-used for balencing
double speedVal = 0;//how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;//difference in speed for each motor-how much should the robot turn (positive=turn right, negative=turn left)
double targetPitch = 0.000;//what angle the balencing control loop should aim for the robot to be at, the output of the speed control loop
double kPA, kIA, kDA = 0.0000;//PID constants for the Angle control loop
double kPS, kIS, kDS = 0.0000;//PID constants for the Speed control loop
PID PIDA(&pitch, &motorSpeedVal, &targetPitch, kPA, kIA, kDA, DIRECT);//setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, kPS, kIA, kDA, DIRECT);//setup the Speed PID loop
WiFiServer server(80);//a wifi server on port 80
void IRAM_ATTR onLeftStepTimer() { //Interrupt function called by timer
  if ((leftMotorSpeed >= 0) != leftForwardBl) {//if going forwards, but last interrupt was going backwards
    if (leftMotorSpeed >= 0) {//if now going forwards
      digitalWrite(leftDirPin, HIGH);//signal the motor controller to go forwards now
    } else {//if now going backwards
      digitalWrite(leftDirPin, LOW);//signal the motor controller to go backwards now
    }
    leftForwardBl = (leftMotorSpeed >= 0);//save if the motor was going forwards for next time
    //delay for 60 clock cycles which at 240MHZ should be 250 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }
  rmt_write_items(configL.channel, itemsL, 1, 0);//start pulse
}
void IRAM_ATTR onRightStepTimer() { //Interrupt function called by timer
  if ((rightMotorSpeed >= 0) != rightForwardBl) {//if going forwards, but last interrupt was going backwards
    if (rightMotorSpeed >= 0) {//if now going forwards
      digitalWrite(rightDirPin, HIGH);//signal the motor controller to go forwards now
    } else {//if now going backwards
      digitalWrite(rightDirPin, LOW);//signal the motor controller to go backwards now
    }
    rightForwardBl = (rightMotorSpeed >= 0);//save if the motor was going forwards for next time
    //delay for 60 clock cycles which at 240MHZ should be 250 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }
  rmt_write_items(configR.channel, itemsR, 1, 0);//start pulse
}
void setup() {//this function is run once when the esp32 turns on and can be used to set up other
  Serial.begin(2000000);//for debug Set the serial monitor to the same value or you will see nothing or gibberish.
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  leftStepTimer = timerBegin(2, 80, true); // 80Mhz / 80  = 1Mhz, 1microsecond
  rightStepTimer = timerBegin(3, 80, true); // 80Mhz / 80  = 1Mhz, 1microsecond
  timerAttachInterrupt(leftStepTimer, &onLeftStepTimer, true); // attach the interrupttimerAttachInterrupt(directionDelayTimer, &amp;amp;onDirectionDelayTimer, true);// attach the interrupt
  timerAttachInterrupt(rightStepTimer, &onRightStepTimer, true); // attach the interrupttimerAttachInterrupt(directionDelayTimer, &amp;amp;onDirectionDelayTimer, true);// attach the interrupt
  timerAlarmWrite(leftStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
  timerAlarmWrite(rightStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
  setupStepperRMTs();
  PIDA.SetMode(MANUAL);//PID loop off
  PIDS.SetMode(MANUAL);//PID loop off
  PIDA.SetSampleTime(1);//tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(1);//tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDA.SetOutputLimits(-MAX_TIP, MAX_TIP);
  PIDS.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  setupMPU6050();//this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();//this function averages some gyro readings so later the readings can be calibrated to zero. This function counts on the robot being still, so the robot needs to be powered on while lying on the ground
  DBserialPrintCurrentCore("setup");//print what core this code is running on. see the declaration of this function lower down for details
  WiFi.softAP(robotSSID, robotPass);//start wifi network, code may need to be added after this to wait for it to start
  IPAddress myIP = WiFi.softAPIP();//get the robot's IP address, it should be the same every time
  Serial.print("AP IP address: ");//Serial.print sends information to your computer over usb, it is useful for making the program show you what it's doing
  Serial.println(myIP);            //print the robot's IP address over serial to find what it is. it should be the same each time. this IP address will probably need to be coded into the DS
  server.begin();                  //start the server
  Serial.println("Server started");//Serial.println is like Serial.print but it adds a newline (enter) so you don't get a single unreadable line
}
void loop() {//this function runs over and over and is where main code is usuall put. I think the esp32 runs this function on core 1. the balencing control loop will be here, with the goal of keeping this function running as fast as possible
  readMPU6050();//read the orientation board
  if (abs(pitch) > MAX_TIP) {//if the robot tilts forwards or backwards beyond the MAX_TIP constant...
    tipped = true;//...set the tipped boolean to true
  } else {
    tipped = false;//...otherwise set it to false
  }
  if (tipped) {//if tipped is true... (you don't have to say ==true, it is assumed)
    robotEnabled = false;//...disable the robot. This is so if the robot falls over, instead of running at full speed trying to right itself as its top skids along the ground, it just disables and can't be enabled until it is righted
    leftMotorSpeed = 0; //don't move
    rightMotorSpeed = 0; //don't move
  }
  if (robotEnabled) {//run the following code if the robot is enabled
    if (!wasRobotEnabled) {//the robot wasn't enabled, but now it is, so this must be the first loop since it was enabled. re set up anything you might want to
      //TODO: turn on stepper motors
      PIDA.SetMode(AUTOMATIC);//turn on the PID
      PIDS.SetMode(AUTOMATIC);//turn on the PID
    }
    PIDA.Compute();//compute the PID, it changes the variables you set it up with earlier.
    PIDS.Compute();//compute the PID, it changes the variables you set it up with earlier.
    leftMotorSpeed = constrain(motorSpeedVal + turnSpeedVal, -MAX_SPEED, MAX_SPEED); //combine motor speed and turn to find the speed the left motor should go
    rightMotorSpeed = constrain(motorSpeedVal - turnSpeedVal, -MAX_SPEED, MAX_SPEED); //combine motor speed and turn to find the speed the right motor should go
    if (abs(leftMotorSpeed) >= 1) {
      timerAlarmWrite(leftStepTimer, 1000000 / leftMotorSpeed, true); // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(leftStepTimer, 10000000000000000, true); //practically never
    }
    if (abs(rightMotorSpeed) >= 1) {
      timerAlarmWrite(rightStepTimer, 1000000 / rightMotorSpeed, true); // 1Mhz / # =  rate
    } else {
      timerAlarmWrite(rightStepTimer, 10000000000000000, true); // practically never
    }
  } else {//run the following code if the robot is disabled. this code should turn off anything that moves
    PIDA.SetMode(MANUAL);//turn the PID off
    PIDS.SetMode(MANUAL);//turn the PiD off
    timerAlarmWrite(leftStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
    timerAlarmWrite(rightStepTimer, 10000000000000000, true); // 1Mhz / # =  rate
    leftMotorSpeed = 0; //don't move
    rightMotorSpeed = 0; //don't move
    //TODO: turn off stepper motors
    delay(2000);
    DBserialPrintCurrentCore("(disabled) loop");//print debugging test of what core the loop is running on
  }
  wasRobotEnabled = robotEnabled;
}
void setupStepperRMTs() {
  configL.rmt_mode = RMT_MODE_TX;
  configL.channel = RMT_CHANNEL_0;
  configL.gpio_num = leftStepPin;
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
  configR.gpio_num = rightStepPin;
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
void WiFiEvent(WiFiEvent_t event) {//this function is hopefully called automatically when something wifiy happens
  DBserialPrintCurrentCore("wifi event");
  Serial.print("wifi event: ");
  Serial.println(event);
  Serial.print("WiFi.RSSI=");
  Serial.println(WiFi.RSSI());//Recieved Signal Strength Indicator, less negative numbers mean a stronger recieved signal
  WiFiClient client = server.available();
  if (client) {                           //now we're trying to print any data we got over wifi. It would be nice if WiFiEvent can be used for our wifi code. hopefully it gets triggered when a message is recieved
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  }
}
void setupMPU6050() {//start I2C communication and send commands to set up the MPU6050.  A command is set by starting a transmission, writing a byte (written here in hexadecimal) to signal what register should be changed, and then sending a new register value
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
  oAX = Wire.read() << 8 | Wire.read();//read two bytes and put them together into a sixteen bit integer value
  oAY = Wire.read() << 8 | Wire.read();
  oAZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();//throw away temperature, it's annoying they put it in the middle here
  oRX = Wire.read() << 8 | Wire.read();
  oRY = Wire.read() << 8 | Wire.read();
  oRZ = Wire.read() << 8 | Wire.read();
  oDPSX = (oRX - oRX0) * 1000000.00 / 32766;//zero gyro with offset values recorded on startup and convert to degrees per second
  oDPSY = (oRY - oRY0) * 1000000.00 / 32766;//zero gyro with offset values recorded on startup and convert to degrees per second
  oDPSZ = (oRZ - oRZ0) * 1000000.00 / 32766;//zero gyro with offset values recorded on startup and convert to degrees per second
  if (micros() > lastCalcedMPU6050) {//try to handle micros' long overflow in a harmless way
    lastCalcedMPU6050 = micros() - 10;
  }
  pitch = .99 * ((pitch - pitchOffset) + oDPSX * (micros() - lastCalcedMPU6050) / 1000000.000) + .01 * (degrees(atan2(oAY, oAZ)) - pitchOffset); //complementary filter combines gyro and accelerometer tilt data in a way that takes advantage of short term accuracy of the gyro and long term accuracy of the accelerometer
  lastCalcedMPU6050 = micros();//record time of last calculation so we know next time how much time has passed (how much time to integrate rotation rate for)
}
void zeroMPU6050() {//find how much offset each gyro axis has to zero out drift. should be run on startup (when robot is still)
  oRX0 = 0; oRY0 = 0; oRZ0 = 0;
  for (int i = 0; i < 50; i++) {//run the following code 50 times so we can get many measurements to average into an offset value
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    oAX = Wire.read() << 8 | Wire.read();//same reading code as earlier
    oAY = Wire.read() << 8 | Wire.read();
    oAZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();//throw away temperature
    oRX = Wire.read() << 8 | Wire.read();
    oRY = Wire.read() << 8 | Wire.read();
    oRZ = Wire.read() << 8 | Wire.read();
    oRX0 += oRX;//add all the reads together
    oRY0 += oRY;
    oRZ0 += oRZ;
    delay(10 + i / 5);//add some time between reads, changing the delay each time a bit to be less likely to be thrown by a periodic oscillation
  }
  oRX0 /= 50;//devide by the number of reads that were taken to get an average value
  oRY0 /= 50;
  oRZ0 /= 50;
}
boolean parseBl(byte * arrayPointer, int* i) { //declare a function that returns a boolean and will be given the location of an array and what element of the array to start at
  byte msg = *(arrayPointer + *i); //read the byte at the location and element given
  if (msg == '0') {//if the number is 0...
    return false;//...return false
  }
  if (msg == '1') {//if the number is one...
    return true;//...return true
  }
  i++;
  return false;//if anything else, default to false
}
byte parseBy(byte * arrayPointer, int* i) { //declare a function that returns a byte and will be given the location of an array and what element of the array to start at
  byte msg = *(arrayPointer + *i); //read the byte from the array given at the location given (kind of a silly function but it will be nice for consistency between other data types
  i++;//increment the counter for the next value
  return msg;//and return it
}
int parseIn(byte * arrayPointer, int* i) { //declare a function that returns an int and will be given the location of an array and what element of the array to start at
  union {//this lets us translate between two variables (equal size, but one's two bytes in an array, and one's a two byte int  Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[2];
    int v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the int
  d.b[0] = *(arrayPointer + *i); //read the first byte
  i++;//increment i to the location of the second byte
  d.b[1] = *(arrayPointer + *i); //read the second byte
  i++;//shift i the second time so it's ready for the next function
  return d.v;//return the int form of union d
}
float parseFl(byte * arrayPointer, int* i) { //declare a function that returns a (4 byte) float and will be given the location of an array and what element of the array to start at
  union {//this lets us translate between two variables (equal size, but one's 4 bytes in an array, and one's a 4 byte float Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the float
  d.b[0] = *(arrayPointer + *i); //read the first byte from the array
  i++;//increment i to the next position in the array
  d.b[1] = *(arrayPointer + *i);//read the second byte from the array
  i++;//i to 3rd position now
  d.b[2] = *(arrayPointer + *i);//read the third byte from the array
  i++;//i to 4th position now
  d.b[3] = *(arrayPointer + *i);//read the fourth (last) byte from the array
  i++;//shift i once more so it's ready for the next function (at the position of the start of the next value)
  return d.v;//return the float form of union d
}

void arrayBl(boolean msg, byte * arrayPointer, int* i) { //declare a function that returns a boolean and will be given the location of an array and what element of the array to start at
  if (msg) {//if true
    *(arrayPointer + *i) = 1;//set the ith element of the array to 1
  } else {//if false
    *(arrayPointer + *i) = 0;//set the ith element of the array to 0
  }
  i++;//increment i for the next position in the array for the next piece of data
}
void arrayBy(byte msg, byte * arrayPointer, int* i) { //declare a function that returns a byte and will be given the location of an array and what element of the array to start at
  *(arrayPointer + *i) = msg;//set the ith element of the array to the passed in msg byte
  i++;//increment the counter for the next value
}
void arrayIn(int msg, byte * arrayPointer, int* i) { //declare a function that returns an int and will be given the location of an array and what element of the array to start at
  union {//this lets us translate between two variables (equal size, but one's two bytes in an array, and one's a two byte int  Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[2];
    int v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the int
  d.v = msg;
  *(arrayPointer + *i) = d.b[0];//set the first byte of the array equal to the first byte of the union
  i++;//increment i to the location of the second byte
  *(arrayPointer + *i) = d.b[1]; //set the second byte
  i++;//shift i the second time so it's ready for the next function
}
void arrayFl(float msg, byte * arrayPointer, int* i) { //declare a function that returns a (4 byte) float and will be given the location of an array and what element of the array to start at
  union {//this lets us translate between two variables (equal size, but one's 4 bytes in an array, and one's a 4 byte float Reference for unions: https://www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;//d is the union, d.b acceses the byte array, d.v acceses the float
  d.v = msg;//put the value into the union
  *(arrayPointer + *i) = d.b[0];//set the first byte of the array equal to the first byte of the union
  i++;//increment i to the next position in the array
  *(arrayPointer + *i) = d.b[1];//set second byte
  i++;//i to 3rd position now
  *(arrayPointer + *i) = d.b[2];//set third byte
  i++;//i to 4th position now
  *(arrayPointer + *i) = d.b[3];//set fourth byte
  i++;//shift i once more so it's ready for the next function (at the position of the start of the next value)
}
