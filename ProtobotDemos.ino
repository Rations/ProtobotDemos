#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

const int VACUUM_PIN      = 2;
const int JOYSTICK_LX_PIN = A0;
const int JOYSTICK_LY_PIN = A1;
const int JOYSTICK_RX_PIN = A2;
const int JOYSTICK_RY_PIN = A3;

const int JOYSTICK_MIN    = 0;
const int JOYSTICK_MAX    = 1023;
const int ARM_SPEED_MIN   = 0;
const int ARM_SPEED_ZERO  = 90;
const int ARM_SPEED_MAX   = 180;
const int BASE_SPEED_MIN  = -255;
const int BASE_SPEED_MAX  = 255;   
const int LIFT_SPEED_MIN  = -255;
const int LIFT_SPEED_MAX  = 255;

int readAxis(int thisAxis, int low, int high) {
  int reading = analogRead(thisAxis);
  reading = map(reading, JOYSTICK_MIN, JOYSTICK_MAX, low, high);
  int center = (high - low) / 2 + low;
  int distance = reading - center;
  int threshold = (high - low) / 6; 
  if (abs(distance) < threshold) {
    distance = 0;
  }
  return distance;
}
                          
// Specify Arduino pins for arm servo and PS2 controller connections. 
#define ARM_SERVO_PIN     10

// PS2 joystick characteristics.
#define REFRESH       5     // Controller refresh rate (ms)



// Declare servo, motor, and PS2 controller objects.
Servo armServo;
Adafruit_MotorShield motorShield  = Adafruit_MotorShield();
Adafruit_DCMotor *liftMotor1      = motorShield.getMotor(1);
Adafruit_DCMotor *liftMotor2      = motorShield.getMotor(2); 
Adafruit_DCMotor *baseMotor       = motorShield.getMotor(3);
Adafruit_DCMotor *vacuum          = motorShield.getMotor(4);
#define VACUUM_DEADZONE 200                                     // Prevents unintended switching when button is depressed for up to t = VACUUM_DEADZONE ms. 
int vacuumState = LOW;                                       // Allows vacuum to be switched on and off alternatingly. 
long    lastTime = 0;                                              // Stores last time vacuum switch (R2) was depressed.

void setup() {
  pinMode(VACUUM_PIN, INPUT);
  pinMode(JOYSTICK_LX_PIN, INPUT);
  pinMode(JOYSTICK_LY_PIN, INPUT);
  pinMode(JOYSTICK_RX_PIN, INPUT);
  pinMode(JOYSTICK_RY_PIN, INPUT);
  motorShield.begin();
  armServo.attach(ARM_SERVO_PIN);
  armServo.write(ARM_SPEED_ZERO);
  liftMotor1 -> run(RELEASE);
  liftMotor2 -> run(RELEASE);
  baseMotor -> run(RELEASE);
  vacuum -> run(RELEASE);
}
 
void loop() {
  // Read vertical-axis inputs from right joystick to extend/retract arm.
  int joystickRY = readAxis(JOYSTICK_RY_PIN, ARM_SPEED_MIN, ARM_SPEED_MAX);
  armServo.write(joystickRY);

  // Read horizontal-axis inputs from left joystick to rotate base.
  int joystickLX = readAxis(JOYSTICK_LX_PIN, BASE_SPEED_MIN, BASE_SPEED_MAX);
  if (joystickLX != 0) {
    baseMotor -> setSpeed(abs(joystickLX));
    if (joystickLX > 0) { 
      baseMotor -> run(FORWARD);
    } else {
      baseMotor -> run(BACKWARD);
    } 
  } else {
      baseMotor -> run(RELEASE);
  }

  // Read vertical-axis inputs from left joystick to raise/lower lift.
  int joystickLY = readAxis(JOYSTICK_LY_PIN, LIFT_SPEED_MIN, LIFT_SPEED_MAX);
  if (joystickLY != 0) {
    liftMotor1 -> setSpeed(abs(joystickLY));
    liftMotor2 -> setSpeed(abs(joystickLY));
    if (joystickLY > 0) { 
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

  // Read vacuum button. If vacuum button is depressed and at least t = VACUUM_DEADZONE ms has elapsed since last depression, switch vacuum on/off as necessary.
  long currentTime = millis();
  int currentVacuumState = digitalRead(VACUUM_PIN);
  if (currentVacuumState != vacuumState && currentTime - lastTime > VACUUM_DEADZONE) {
    if (currentVacuumState == LOW) {
      vacuum -> run(FORWARD);                 
      vacuum -> setSpeed(255);
      vacuumState = HIGH;
    } else {
      vacuum -> run(RELEASE);
      vacuumState = LOW;
    }
    lastTime = millis();
  }
  delay(REFRESH);
}
