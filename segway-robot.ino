void fscmCFSetupGyro() {
  Wire.begin();//////////////////////setup gy521
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
  Wire.write(0x04);//clock devider
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x00);//buffering
  Wire.endTransmission(true);///////end setup gy521
}
void fscmCFReadGyro() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  cfAX = Wire.read() << 8 | Wire.read();
  cfAY = Wire.read() << 8 | Wire.read();
  cfAZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();//throw away temperature
  cfGX = Wire.read() << 8 | Wire.read();
  cfGY = Wire.read() << 8 | Wire.read();
  cfGZ = Wire.read() << 8 | Wire.read();
  GDSX = (cfGX - GX0) * 1000.00 / 32766;
  GDSY = (cfGY - GY0) * 1000.00 / 32766;
  GDSZ = (cfGZ - GZ0) * 1000.00 / 32766;
  fscmCPitch = .99 * (fscmCPitch + GDSX * (millis() - lastCalcedGyro) / 1000.000) + .01 * degrees(atan2(cfAY, cfAZ));
  fscmCRoll = .99 * (fscmCRoll + GDSY * (millis() - lastCalcedGyro) / 1000.000) + .01 * degrees(atan2(-cfAX, cfAZ));
  lastCalcedGyro = millis();
}
