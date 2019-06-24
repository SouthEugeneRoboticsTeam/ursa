int16_t oAX, oAY, oAZ, oRX, oRY, oRZ, oX0, oY0, oZ0 = 0; //for MPU6050
unsigned long lastCalcedMPU6050 = 0;
float pitch = 0.000;
void setup() {
  setupGyro(MPU6050);
  zeroGyro(MPU6050);
}
void loop() { //core 1
}
void setupMPU6050() {
  Wire.begin();//////////////////////setup mpu6050
  Wire.setClock(400000L);
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
  Wire.write(0x04);//clock divider
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x00);//buffering
  Wire.endTransmission(true);///////end setup mpu6050
}
void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  oAX = Wire.read() << 8 | Wire.read();
  oAY = Wire.read() << 8 | Wire.read();
  oAZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();//throw away temperature
  oRX = Wire.read() << 8 | Wire.read();
  oRY = Wire.read() << 8 | Wire.read();
  oRZ = Wire.read() << 8 | Wire.read();
  oDPSX = (oRX - oGX0) * 1000.00 / 32766;//convert to degrees per second
  oDPSY = (oRY - oGY0) * 1000.00 / 32766;
  oDPSZ = (oRZ - oGZ0) * 1000.00 / 32766;
  pitch = .99 * (pitch + oDPSX * (millis() - lastCalcedMPU6050) / 1000.000) + .01 * degrees(atan2(oAY, oAZ));
  lastCalcedMPU6050 = millis();
}
void zeroMPU6050() {
  GX0 = 0; GY0 = 0; GZ0 = 0;
  for (int i = 0; i < 50; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    oAX = Wire.read() << 8 | Wire.read();
    oAY = Wire.read() << 8 | Wire.read();
    oAZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();//throw away temperature
    oRX = Wire.read() << 8 | Wire.read();
    oRY = Wire.read() << 8 | Wire.read();
    oRZ = Wire.read() << 8 | Wire.read();
    oGX0 += oRX;
    oGY0 += oRY;
    oGZ0 += oRZ;
    delay(10 + i / 5);
  }
  oGX0 /= 50;
  oGY0 /= 50;
  oGZ0 /= 50;
}
