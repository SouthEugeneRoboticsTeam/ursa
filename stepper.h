#ifndef stepper_h
#define stepper_h
#include "Arduino.h"
#include "driver/rmt.h"

hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;

rmt_config_t leftConfig;    // settings for RMT pulse for stepper motor
rmt_item32_t leftItems[1];  // holds definition of pulse for stepper motor
rmt_config_t rightConfig;
rmt_item32_t rightItems[1];

volatile boolean rightForwardBl = false;  // was the motor moving forwards last time the interrupt was called
volatile boolean leftForwardBl = false;

#endif
