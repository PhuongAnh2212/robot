// Define motor control pins
int enL = 7;   // Left motor PWM
int inL1 = 8;  // Left motor direction 1
int inL2 = 9;  // Left motor direction 2
int enR = 12;  // Right motor PWM
int inR1 = 10; // Right motor direction 1
int inR2 = 11; // Right motor direction 2

// Encoder pins
int enLA = 2;  // Left encoder A (interrupt pin)
int enRA = 18; // Right encoder A (interrupt pin)

// Define motor speeds (PWM values)
int leftMotorSpeed = 255; // Full speed
int rightMotorSpeed = 153; // Adjusted speed for turning

// Define time to move in circle (milliseconds)
unsigned long circleTime = 10000; // 10 seconds for full circle

void setup() {
  // Initialize motor control pins
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  
  // Set initial motor directions
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
  
  // Start motors
  analogWrite(enL, leftMotorSpeed);
  analogWrite(enR, rightMotorSpeed);
  
  // Delay to run the robot in a circle for the specified time
  delay(circleTime);
  
  // Stop motors
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}

void loop() {
  // Nothing to do here
}
