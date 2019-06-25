#include <Wire.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <AccelStepper.h>
const char *robotSSID = "SERT_URSA_0";
const char *robotPass = "sert2521";
boolean robotEnabled = false;
boolean enable = false;
boolean tipped = false;
int16_t oAX, oAY, oAZ, oRX, oRY, oRZ, oRX0, oRY0, oRZ0 = 0; //for MPU6050
unsigned long lastCalcedMPU6050 = 0;
float oDPSX, oDPSY, oDPSZ = 0.000;
float pitch = 0.000;
int leftSpeed = 0;
int rightSpeed = 0;
int motorSpeedVal = 0;
int speedVal = 0;
int turnVal = 0;
float targetPitch = 0.000;
float PA, IA, DA, PS, IS, DS = 0.0000;
PID PIDA(&pitch, &motorSpeedVal, &targetPitch, PA, IA, DA, DIRECT);
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, PS, IS, DS, DIRECT);
AccelStepper leftStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper rightStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
WiFiServer server(80);
void setup() {
  PIDA.SetMode(AUTOMATIC);
  PIDS.SetMode(AUTOMATIC);
  PIDA.SetSampleTime(1);
  PIDS.SetSampleTime(1);
  Serial.begin(2000000);//for debug
  setupMPU6050();
  zeroMPU6050();
  leftStepper.setMaxSpeed(4000);//test
  rightStepper.setMaxSpeed(4000);//test
  WiFi.softAP(robotSSID, robotPass);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
}
void loop() { //core 1
  readMPU6050();
  PIDA.Compute();
  PIDS.Compute();
  leftStepper.setSpeed(motorSpeedVal + turnVal);
  rightStepper.setSpeed(motorSpeedVal - turnVal);
  leftStepper.run();
  rightStepper.run();
}
void DBserialPrintCurrentCore(String msg) {
  Serial.print(msg);
  Serial.print(" running on core #");
  Serial.println(xPortGetCoreID());
}
void WiFiEvent(WiFiEvent_t event) {
  DBserialPrintCurrentCore("wifi event");
  Serial.print("wifi event: ");
  Serial.println(event);
  Serial.print("WiFi.RSSI=");
  Serial.println(WiFi.RSSI());
  WiFiClient client = server.available();
  if (client) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  }
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
  oDPSX = (oRX - oRX0) * 1000.00 / 32766;//convert to degrees per second
  oDPSY = (oRY - oRY0) * 1000.00 / 32766;
  oDPSZ = (oRZ - oRZ0) * 1000.00 / 32766;
  pitch = .99 * (pitch + oDPSX * (millis() - lastCalcedMPU6050) / 1000.000) + .01 * degrees(atan2(oAY, oAZ));
  lastCalcedMPU6050 = millis();
}
void zeroMPU6050() {
  oRX0 = 0; oRY0 = 0; oRZ0 = 0;
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
    oRX0 += oRX;
    oRY0 += oRY;
    oRZ0 += oRZ;
    delay(10 + i / 5);
  }
  oRX0 /= 50;
  oRY0 /= 50;
  oRZ0 /= 50;
}
