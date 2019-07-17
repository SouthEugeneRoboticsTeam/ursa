void IRAM_ATTR onLeftStepTimer() {  // Interrupt function called by timer
  digitalWrite(LEFT_STEP_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(LEFT_STEP_PIN, LOW);
}

void IRAM_ATTR onRightStepTimer() {  // Interrupt function called by timer
  digitalWrite(RIGHT_STEP_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(RIGHT_STEP_PIN, LOW);
}

void setupStepperTimers() {
  leftStepTimer = timerBegin(2, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  rightStepTimer = timerBegin(3, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  timerAttachInterrupt(leftStepTimer, &onLeftStepTimer, true);
  timerAttachInterrupt(rightStepTimer, &onRightStepTimer, true);
  timerAlarmWrite(leftStepTimer, 100000000000000000, true);  // 1Mhz / # =  rate // practically never
  timerAlarmWrite(rightStepTimer, 100000000000000000, true);  // 1Mhz / # =  rate
  timerAlarmEnable(leftStepTimer);
  timerAlarmEnable(rightStepTimer);
}
