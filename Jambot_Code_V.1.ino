#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <PS2X_lib.h>
                            
// Specify Arduino pins for arm servo and PS2 controller connections. 
#define ARM_SERVO_PIN     10
#define PS2_CLOCK_PIN     9
#define PS2_COMMAND_PIN   7
#define PS2_ATTENTION_PIN 8
#define PS2_DATA_PIN      6

// PS2 joystick characteristics.
#define PS2_REFRESH       10     // Controller refresh rate (ms)
#define Y_JOYSTICK_ZERO   127   // Joystick midpoint value
#define X_JOYSTICK_ZERO   128
#define JOYSTICK_RANGE    124   // (128 - JOYSTICK DEADZONE)*2
#define JOYSTICK_DEADZONE 4

// Arm servo characteristics. 0 = full speed in one direction, 90 = zero speed, 180 = full speed in opposite direction.
#define ARM_SPEED_ZERO    90
#define ARM_SPEED_RANGE   180
float ARM_SPEED_SCALE =   ARM_SPEED_RANGE / JOYSTICK_RANGE;

#define BASE_SPEED_ZERO    0
#define BASE_SPEED_RANGE   255   
float BASE_SPEED_SCALE =   BASE_SPEED_RANGE / JOYSTICK_RANGE;

#define LIFT_SPEED_ZERO    0
#define LIFT_SPEED_RANGE   255
float LIFT_SPEED_SCALE =   LIFT_SPEED_RANGE / JOYSTICK_RANGE;

// Global variables storing servo speeds; initialize to zero speed.  
float armSpeed  = ARM_SPEED_ZERO;
float baseSpeed = BASE_SPEED_ZERO;
float liftSpeed = LIFT_SPEED_ZERO;

// Declare servo, motor, and PS2 controller objects.
PS2X  ps2;
Servo armServo;

Adafruit_MotorShield motorShield  = Adafruit_MotorShield();
Adafruit_DCMotor *liftMotor1      = motorShield.getMotor(1);
Adafruit_DCMotor *liftMotor2      = motorShield.getMotor(2); 
Adafruit_DCMotor *baseMotor       = motorShield.getMotor(3);
Adafruit_DCMotor *vacuum          = motorShield.getMotor(4);
#define VACUUM_DEADZONE 200                                     // Prevents unintended switching when button is depressed for up to t = VACUUM_DEADZONE ms. 
boolean vacuumOn = false;                                       // Allows vacuum to be switched on and off alternatingly. 
long    lastTime = 0;                                              // Stores last time vacuum switch (R2) was depressed.

void setup() {
  Serial.begin(9600);
  motorShield.begin();
  armServo.attach(ARM_SERVO_PIN);
  
  // Set up PS2 controller; loop until ready.
  byte ps2Status;
  do {
    ps2Status = ps2.config_gamepad(PS2_CLOCK_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_DATA_PIN);
  } while (ps2Status == 1);
  delay(100);
}
 
void loop() {
  ps2.read_gamepad();
  
  // Read vertical-axis inputs from right joystick to extend/retract arm.
  float joystickRY = (float)Y_JOYSTICK_ZERO - ps2.Analog(PSS_RY);
  if (abs(joystickRY) > JOYSTICK_DEADZONE) {
    armSpeed = ARM_SPEED_ZERO + joystickRY * ARM_SPEED_SCALE;
    armServo.write(armSpeed);
  } else {
    armServo.write(ARM_SPEED_ZERO);
  }

  // Read horizontal-axis inputs from left joystick to rotate base.
  float joystickLX = (float)ps2.Analog(PSS_LX) - X_JOYSTICK_ZERO;
  if (joystickLX != 0) {
    baseSpeed = BASE_SPEED_ZERO + joystickLX * BASE_SPEED_SCALE + 1;
    baseMotor -> setSpeed(abs(baseSpeed));
    if (baseSpeed > 0) { 
      baseMotor -> run(FORWARD);
    } 
    if (baseSpeed < 0) {
      baseMotor -> run(BACKWARD);
    } 
  } else {
      baseMotor -> run(RELEASE);
  }

  // Read vertical-axis inputs from left joystick to raise/lower lift.
  float joystickLY = (float)Y_JOYSTICK_ZERO - ps2.Analog(PSS_LY);
  if (joystickLY != 0) {
    liftSpeed = LIFT_SPEED_ZERO + joystickLY * LIFT_SPEED_SCALE + 1;
    liftMotor1 -> setSpeed(abs(liftSpeed));
    liftMotor2 -> setSpeed(abs(liftSpeed));
    if (liftSpeed > 0) { 
      liftMotor1 -> run(FORWARD);
      liftMotor2 -> run(FORWARD);
    } else {
      liftMotor1 -> run(BACKWARD);
      liftMotor2 -> run(BACKWARD);
    } 
  } else {
      liftMotor1 -> run(RELEASE);
      liftMotor2 -> run(RELEASE);
  }

  // Read R2. If R2 is depressed and at least t = VACUUM_DEADZONE ms has elapsed since last depression, switch vacuum on/off as necessary.
  long currentTime = millis();
  if (ps2.Button(PSB_R2) && currentTime - lastTime > VACUUM_DEADZONE) {
    if (!vacuumOn) {
      vacuum -> run(FORWARD);                 
      vacuum -> setSpeed(255);
    } else {
      vacuum -> run(RELEASE);
    }
    vacuumOn = !vacuumOn;
    lastTime = millis();
  }
  delay(PS2_REFRESH);
}
