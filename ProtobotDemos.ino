// Declare servo, motor, and PS2 controller objects.
Servo armServo;
Adafruit_MotorShield motorShield  = Adafruit_MotorShield();
Adafruit_DCMotor *liftMotor1      = motorShield.getMotor(1);
Adafruit_DCMotor *liftMotor2      = motorShield.getMotor(2); 
Adafruit_DCMotor *baseMotor       = motorShield.getMotor(3);
Adafruit_DCMotor *vacuum          = motorShield.getMotor(4);
#define VACUUM_DEADZONE 200                                     // Prevents unintended switching when button is depressed for up to t = VACUUM_DEADZONE ms. 
int vacuumState = HIGH;                                       // Allows vacuum to be switched on and off alternatingly. 
long    lastTime = 0;                                              // Stores last time vacuum switch (R2) was depressed.

void setup() {
  Serial.begin(9600);
  pinMode(VACUUM_PIN, INPUT);
  digitalWrite(VACUUM_PIN, HIGH);
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
  //double joystickRY = readAxis(JOYSTICK_RY_PIN, ARM_SPEED_MAX, ARM_SPEED_MIN);
  double joystickRY = (analogRead(JOYSTICK_RY_PIN)*180.0)/1023.0;
  armServo.write((int)joystickRY);

  // Read horizontal-axis inputs from left joystick to rotate base.
  int joystickLX = readAxis(JOYSTICK_LX_PIN, BASE_SPEED_MAX, BASE_SPEED_MIN);
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
  int joystickLY = readAxis(JOYSTICK_LY_PIN, LIFT_SPEED_MAX, LIFT_SPEED_MIN);
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
  Serial.println(digitalRead(VACUUM_PIN));
  int currentVacuumState = digitalRead(VACUUM_PIN);
  if (currentVacuumState == LOW && currentTime - lastTime > VACUUM_DEADZONE) {
    if (vacuumState == LOW) {
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

