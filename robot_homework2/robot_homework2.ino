// Motor pins
int enL = 7;  // Left motor PWM
int inL1 = 8; // Left motor direction 1
int inL2 = 9; // Left motor direction 2

int enR = 12; // Right motor PWM
int inR1 = 10; // Right motor direction 1
int inR2 = 11; // Right motor direction 2

// Encoder pins
int enLA = 2;  // Left encoder A
int enRA = 18; // Right encoder A

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
int prevLeftEnCount = 0;
int prevRightEnCount = 0;


// Practice question 1:
// const int targetCount = 350 * 2 / 3; // Number of pulses for one revolution

// Practice question 2:
const int targetCount = 5500;

bool leftMovementComplete = false;
bool rightMovementComplete = false;
bool bothMotorsStopped = false; // Flag to ensure "Both motors stopped" is printed only once

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // Set motor pins as outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Set encoder pins as inputs
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, CHANGE);

  // Turn off motors initially
  stop();
}

void loop() {
  // Start both motors if not yet complete
  if (!leftMovementComplete || !rightMovementComplete) {
    goForward(255); // Run both motors at the same time
  }

  // Process left motor
  if (!leftMovementComplete && leftEnCount != prevLeftEnCount) {
    Serial.print("Left count: ");
    Serial.println(leftEnCount);

    if (leftEnCount >= targetCount) {
      leftMovementComplete = true;
      stopLeftMotor();
      Serial.println("Left motor target reached.");
    }

    prevLeftEnCount = leftEnCount;
  }

  // Process right motor
  if (!rightMovementComplete && rightEnCount != prevRightEnCount) {
    Serial.print("Right count: ");
    Serial.println(rightEnCount);

    if (rightEnCount >= targetCount) {
      rightMovementComplete = true;
      stopRightMotor();
      Serial.println("Right motor target reached.");
    }

    prevRightEnCount = rightEnCount;
  }

  // If both motors are done and not already stopped, stop everything and print the message once
  if (leftMovementComplete && rightMovementComplete && !bothMotorsStopped) {
    stop();
    Serial.println("Both motors stopped.");
    bothMotorsStopped = true; // Set flag to prevent printing again
  }

  // Delay for stability
  delay(100);
}

void goForward(int speed) {
  // Set PWM for motors
  analogWrite(enL, speed);
  analogWrite(enR, speed);

  // Set motor direction for forward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void stopLeftMotor() {
  // Turn off left motor
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void stopRightMotor() {
  // Turn off right motor
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void stop() {
  // Turn off both motors
  stopLeftMotor();
  stopRightMotor();
}

void leftEnISRA() {
  leftEnCount++;
}

void rightEnISRA() {
  rightEnCount++;
}
