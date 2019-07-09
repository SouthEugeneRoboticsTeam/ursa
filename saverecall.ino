void recallSettings() {
  byte counter = 0;
  Serial.println("RECALLING");
  kP_angle = EEPROMreadFloat(counter);
  kI_angle = EEPROMreadFloat(counter);
  Serial.println(kI_angle);
  kD_angle = EEPROMreadFloat(counter);
  kP_speed = EEPROMreadFloat(counter);
  kI_speed = EEPROMreadFloat(counter);
  kD_speed = EEPROMreadFloat(counter);
}

void saveSettings() {
  byte counter = 0;
  Serial.println("SAVING");
  EEPROMwriteFloat(kP_angle, counter);
  EEPROMwriteFloat(kI_angle, counter);
  Serial.println(kI_angle);
  EEPROMwriteFloat(kD_angle, counter);
  EEPROMwriteFloat(kP_speed, counter);
  EEPROMwriteFloat(kI_speed, counter);
  EEPROMwriteFloat(kD_speed, counter);
  EEPROM.commit();
  for (int i = 0; i < 63; i++) {
    Serial.print(EEPROM.read(i));
    Serial.print(",");
  }
  Serial.println();
}

boolean EEPROMreadBoolean(byte &pos) {
  byte msg = byte(EEPROM.read(pos));
  pos++;
  return (msg == 1);
}

byte EEPROMreadByte(byte &pos) {
  byte msg = byte(EEPROM.read(pos));
  pos++;
  return msg;
}

int EEPROMreadInt(byte &pos) {
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

float EEPROMreadFloat(byte &pos) {
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

void EEPROMwriteBool(boolean msg, byte &pos) {
  EEPROM.write(pos, msg);
  pos++;
}

void EEPROMwriteByte(byte msg, byte &pos) {
  EEPROM.write(pos, msg);
  pos++;
}

void EEPROMwriteInt(int msg, byte &pos) {
  union {
    byte b[4];
    int v;
  } d;

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    EEPROM.write(pos, d.b[i]);
    pos++;
  }
}

void EEPROMwriteFloat(float msg, byte &pos) {
  union {
    byte b[4];
    float v;
  } d;

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    EEPROM.write(pos, d.b[i]);
    pos++;
  }
}
