#ifndef mpu_h
#define mpu_h
#include "Arduino.h"
#include <Wire.h>  // scl=22 sda=21
#define movementThreshold 34
#define movementMeasurements 15

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ = 0;
int32_t rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;  // "offset" values used to zero the MPU6050 gyro on startup
unsigned long lastCalcedMPU6050 = 0;  // micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;  // rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;  // output (in degrees) from the MPU6050 reading code. negative=forwards, positive=back Pitch matters for self balancing.
float pitchOffset = 0.000;  // subtracted from the output in readMPU6050 so that zero pitch can correspond to balanced, not that the control loop cares. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not respond to perfectly balenced.

#endif
