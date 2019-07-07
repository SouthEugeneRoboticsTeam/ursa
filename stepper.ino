void IRAM_ATTR onLeftStepTimer() {  // Interrupt function called by timer
  if ((leftMotorSpeed >= 0) != leftForwardBl) {  // if direction has changed
    if (leftMotorSpeed >= 0) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
    } else {
      digitalWrite(LEFT_DIR_PIN, LOW);
    }

    leftForwardBl = (leftMotorSpeed >= 0);  // save direction for next time

    // delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }

  rmt_write_items(leftConfig.channel, leftItems, 1, 0);  // start pulse
}

void IRAM_ATTR onRightStepTimer() {  // Interrupt function called by timer
  if ((rightMotorSpeed >= 0) != rightForwardBl) {  // if direction has changed
    if (rightMotorSpeed >= 0) {
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    } else {
      digitalWrite(RIGHT_DIR_PIN, LOW);
    }

    rightForwardBl = (rightMotorSpeed >= 0);  // save direction for next time

    // delay for 72 clock cycles which at 240MHZ should be 300 nanoseconds. this much time is required by the driver chip between any direction change and a step command
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
  }

  rmt_write_items(rightConfig.channel, rightItems, 1, 0);  // start pulse
}

void setupStepperTimers() {
  leftStepTimer = timerBegin(2, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  rightStepTimer = timerBegin(3, 80, true);  // 80Mhz / 80  = 1Mhz, 1microsecond
  timerAttachInterrupt(leftStepTimer, &onLeftStepTimer, true);
  timerAttachInterrupt(rightStepTimer, &onRightStepTimer, true);
  timerAlarmWrite(leftStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate // practically never
  timerAlarmWrite(rightStepTimer, 10000000000000000, true);  // 1Mhz / # =  rate
}

void setupStepperRMTs() {
  leftConfig.rmt_mode = RMT_MODE_TX;
  leftConfig.channel = RMT_CHANNEL_0;
  leftConfig.gpio_num = LEFT_STEP_PIN;
  leftConfig.mem_block_num = 1;
  leftConfig.tx_config.loop_en = 0;
  leftConfig.tx_config.carrier_en = 0;
  leftConfig.tx_config.idle_output_en = 1;
  leftConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  leftConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  leftConfig.clk_div = 80;  // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&leftConfig);
  rmt_driver_install(leftConfig.channel, 0, 0);  // rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  leftItems[0].duration0 = 2;
  leftItems[0].level0 = 1;
  leftItems[0].duration1 = 0;
  leftItems[0].level1 = 0;

  rightConfig.rmt_mode = RMT_MODE_TX;
  rightConfig.channel = RMT_CHANNEL_1;
  rightConfig.gpio_num = RIGHT_STEP_PIN;
  rightConfig.mem_block_num = 1;
  rightConfig.tx_config.loop_en = 0;
  rightConfig.tx_config.carrier_en = 0;
  rightConfig.tx_config.idle_output_en = 1;
  rightConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rightConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  rightConfig.clk_div = 80;  // 80MHx / 80 = 1MHz 0r 1uS per count
  rmt_config(&rightConfig);
  rmt_driver_install(rightConfig.channel, 0, 1);  // rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  rightItems[0].duration0 = 2;
  rightItems[0].level0 = 1;
  rightItems[0].duration1 = 0;
  rightItems[0].level1 = 0;
}
