#include "mpu.h"

// start I2C communication and send commands to set up the MPU6050.
// A command is set by starting a transmission, writing a byte (written here in hexadecimal) to signal what register should be changed,
// and then sending a new register value
void setupMPU6050() {
  Wire.begin();  // setup mpu6050. reference for the mpu6050's registers https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  Wire.setClock(400000L);  // send data at a faster clock speed
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);  // wakeup
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);  // gyro range //18=2000//10=1000
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x19);
  Wire.write(0x02);  // clock divider
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x00);  // buffering
  Wire.endTransmission(true);  // end setup mpu6050
}

void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // location of first byte of data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);  // ask for accel and gyro data bytes
  accelerationX = Wire.read() << 8 | Wire.read();  // read two bytes and put them together into a sixteen bit integer value
  accelerationY = Wire.read() << 8 | Wire.read();
  accelerationZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // throw away temperature, it's annoying they put it in the middle here
  rotationX = Wire.read() << 8 | Wire.read();
  rotationY = Wire.read() << 8 | Wire.read();
  rotationZ = Wire.read() << 8 | Wire.read();
  rotationDPS_X = (rotationX - rotationOffsetX) * 1000.00 / 32766;  // zero gyro with offset values recorded on startup and convert to degrees per second
  rotationDPS_Y = (rotationY - rotationOffsetY) * 1000.00 / 32766;
  rotationDPS_Z = (rotationZ - rotationOffsetZ) * 1000.00 / 32766;

  if (micros() < lastCalcedMPU6050) {  // try to handle micros' long overflow in a harmless way
    lastCalcedMPU6050 = micros() - 10000;
  }

  pitch = COMPLEMENTARY_FILTER_CONSTANT * ((pitch - pitchOffset) + rotationDPS_X * (micros() - lastCalcedMPU6050) / 1000000.000) + (1 - COMPLEMENTARY_FILTER_CONSTANT) * (degrees(atan2(accelerationY, accelerationZ)) - pitchOffset);  // complementary filter combines gyro and accelerometer tilt data in a way that takes advantage of short term accuracy of the gyro and long term accuracy of the accelerometer
  lastCalcedMPU6050 = micros();  // record time of last calculation so we know next time how much time has passed (how much time to integrate rotation rate for)
}

void zeroMPU6050() {  // find how much offset each gyro axis has to zero out drift. should be run on startup (when robot is still)
  do {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    accelerationX = Wire.read() << 8 | Wire.read();  // same reading code as earlier
    accelerationY = Wire.read() << 8 | Wire.read();
    accelerationZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // throw away temperature
    int16_t lastrotationX = Wire.read() << 8 | Wire.read();
    int16_t lastrotationY = Wire.read() << 8 | Wire.read();
    int16_t lastrotationZ = Wire.read() << 8 | Wire.read();
    rotationOffsetX = 0;
    rotationOffsetY = 0;
    rotationOffsetZ = 0;

    for (int i = 0; i < movementMeasurements; i++) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 14, true);
      accelerationX = Wire.read() << 8 | Wire.read();  // same reading code as earlier
      accelerationY = Wire.read() << 8 | Wire.read();
      accelerationZ = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read();  // throw away temperature
      rotationX = Wire.read() << 8 | Wire.read();
      rotationY = Wire.read() << 8 | Wire.read();
      rotationZ = Wire.read() << 8 | Wire.read();
      rotationOffsetX += abs(rotationX - lastrotationX);
      rotationOffsetY += abs(rotationY - lastrotationY);
      rotationOffsetZ += abs(rotationZ - lastrotationZ);
      lastrotationX = rotationX;
      lastrotationY = rotationY;
      lastrotationZ = rotationZ;
      digitalWrite(LED_BUILTIN, i % 2);
      delay(25);
    }
  } while (abs(rotationOffsetX) > movementThreshold * movementMeasurements || abs(rotationOffsetY) > movementThreshold * movementMeasurements || abs(rotationOffsetZ) > movementThreshold * movementMeasurements);

  rotationOffsetX = 0;
  rotationOffsetY = 0;
  rotationOffsetZ = 0;

  for (int i = 0; i < 50; i++) {  // run the following code 50 times so we can get many measurements to average into an offset value
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    accelerationX = Wire.read() << 8 | Wire.read();  // same reading code as earlier
    accelerationY = Wire.read() << 8 | Wire.read();
    accelerationZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // throw away temperature
    rotationX = Wire.read() << 8 | Wire.read();
    rotationY = Wire.read() << 8 | Wire.read();
    rotationZ = Wire.read() << 8 | Wire.read();
    rotationOffsetX += rotationX;  // add all the reads together
    rotationOffsetY += rotationY;
    rotationOffsetZ += rotationZ;
    delay(10 + i / 5);  // add some time between reads, changing the delay each time a bit to be less likely to be thrown by a periodic oscillation
  }

  rotationOffsetX /= 50;  // devide by the number of reads that were taken to get an average value
  rotationOffsetY /= 50;
  rotationOffsetZ /= 50;
}
