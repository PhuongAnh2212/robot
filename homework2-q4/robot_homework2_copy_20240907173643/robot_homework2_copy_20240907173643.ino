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

// Motor speeds (PWM values)
int baseSpeed = 255; // Base speed for both motors
float wheelbase = 0.187; // Wheelbase of the robot in meters
float radius = 1.0; // Radius of the circle in meters

// Calculated speeds
int leftMotorSpeed;
int rightMotorSpeed;

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
  
  // Calculate the speeds for the motors
  leftMotorSpeed = baseSpeed * (1 - wheelbase / (2 * radius));
  rightMotorSpeed = baseSpeed * (1 + wheelbase / (2 * radius));
  
  // Set motor speeds
  analogWrite(enL, leftMotorSpeed);
  analogWrite(enR, rightMotorSpeed);
}

void loop() {
  // The robot will continue to move in a circle until you stop it
  // You can add code to handle stopping or adjusting behavior if needed
}
