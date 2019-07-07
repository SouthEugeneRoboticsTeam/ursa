#ifndef ursa_h
#define ursa_h
#include <PID_v1.h>

#define ROBOT_ID 0  // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 0  // unique configuration of robot which can be used to identify additional features
#define WiFiLossDisableIntervalMillis 100  // if no data packet has been recieved for this number of milliseconds, the robot disables to prevent running away
float COMPLEMENTARY_FILTER_CONSTANT = .9997;  // higher = more gyro based, lower=more accelerometer based
int MAX_SPEED = 4000;  // max speed (in steps/sec) that the motors can run at
float MAX_TIP = 60;  // max angle in degrees the robot will attempt to recover from -- if passed, robot will disable

// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
#define LEFT_STEP_PIN GPIO_NUM_32
#define LEFT_DIR_PIN GPIO_NUM_33
#define RIGHT_STEP_PIN GPIO_NUM_34
#define RIGHT_DIR_PIN GPIO_NUM_35
#define ENS_PIN GPIO_NUM_23  // pin wired to both motor driver chips' ENable pins, to turn on and off motors
#define LED_BUILTIN 2

byte voltage = 0;  // 0v=0 13v=255

// since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexReceive;  // used to check whether receiving tasks can safely change shared variables

boolean robotEnabled = false;     // enable outputs?
boolean wasRobotEnabled = false;  // to know if robotEnabled has changed
boolean enable = false;           // is the DS telling the robot to enable? (different from robotEnabled so the robot can disable when tipped even if the DS is telling it to enable)
boolean tipped = false;

double targetPitch = 0.000;  // what angle the balencing control loop should aim for the robot to be at, the output of the speed control loop
volatile int leftMotorSpeed = 0;  // stepper ticks per second that the left motor is currently doing "volatile" because used in an interrupt
volatile int rightMotorSpeed = 0;
double motorSpeedVal = 0;  // how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balencing, not one for each motor
double speedVal = 0;  // how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;  // (positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;  // PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;  // PID gains for the Speed control loop
PID PIDA(&pitch, &motorSpeedVal, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);  // setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeedVal, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, DIRECT);  // setup the Speed PID loop

#endif
