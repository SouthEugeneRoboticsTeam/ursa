#ifndef wifi_h
#define wifi_h
#include "Arduino.h"
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define maxWifiRecvBufSize 50  // max number of bytes to receive
#define maxWifiSendBufSize 50  // max number of bytes to send

byte numBytesToSend = 0;
// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "sert2521";

WiFiUDP Udp;

volatile byte recvdData[maxWifiRecvBufSize] = {0};  // array to hold data recieved from DS.
volatile boolean receivedNewData = false;  // set true when data gotten, set false when parsed
volatile byte dataToSend[maxWifiSendBufSize] = {0};  // array to hold data to send to DS.
byte numAuxRecv = 0;  // how many bytes of control data for extra things
byte auxRecvArray[12] = {0};  // size of numAuxRecv
byte numSendAux = 0;  // how many bytes of sensor data to send
byte auxSendArray[12] = {0};  // size of numAuxSend
unsigned long lastMessageTimeMillis = 0;

#endif
