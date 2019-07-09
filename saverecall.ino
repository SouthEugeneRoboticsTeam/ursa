void recallSettings() {
  byte counter = 0;
  kP_angle = EEPROM.readFloat(counter);
  counter++;
  kI_angle = EEPROM.readFloat(counter);
  counter++;
  kD_angle = EEPROM.readFloat(counter);
  counter++;
  kP_speed = EEPROM.readFloat(counter);
  counter++;
  kI_speed = EEPROM.readFloat(counter);
  counter++;
  kD_speed = EEPROM.readFloat(counter);
}

void saveSettings() {
  byte counter = 0;
  EEPROM.writeFloat(kP_angle, counter);
  counter++;
  EEPROM.writeFloat(kI_angle, counter);
  counter++;
  EEPROM.writeFloat(kD_angle, counter);
  counter++;
  EEPROM.writeFloat(kP_speed, counter);
  counter++;
  EEPROM.writeFloat(kI_speed, counter);
  counter++;
  EEPROM.writeFloat(kD_speed, counter);
  EEPROM.commit();
}
