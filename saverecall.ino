boolean recallBoolFromEeprom(byte &pos) {
  byte msg = byte(EEPROM.read(pos));
  pos++;
  return (msg == 1);
}

byte recallByteFromEeprom(byte &pos) {
  byte msg = byte(EEPROM.read(pos));
  pos++;
  return msg;
}

int recallIntFromEeprom(byte &pos) {
  union {
    byte b[4];
    int v;
  } d;

  for (int i = 0; i < 4; i++) {
    d.b[i] = byte(EEPROM.read(pos));
    pos++;
  }

  return d.v;
}

float recallFloatFromEeprom(byte &pos) {
  union {
    byte b[4];
    float v;
  } d;

  for (int i = 0; i < 4; i++) {
    d.b[i] = byte(EEPROM.read(pos));
    pos++;
  }

  return d.v;
}

void saveBoolToEeprom(boolean msg, byte &pos) {
  EEPROM.write(pos, msg);
  pos++;
}

void saveByteToEeprom(byte msg, byte &pos) {
  EEPROM.write(pos, msg);
  pos++;
}

void saveIntToEeprom(int msg, byte &pos) {
  union {
    byte b[4];
    int v;
  } d;

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = d.b[i];
    pos++;
  }
}

void saveFloatToEeprom(float msg, byte &pos) {
  union {
    byte b[4];
    float v;
  } d;

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = d.b[i];
    pos++;
  }
}
