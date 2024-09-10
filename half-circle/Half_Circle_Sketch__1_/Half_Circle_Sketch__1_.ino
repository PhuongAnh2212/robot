// Define motor control pins
int enL = 7;   // Left motor PWM
int inL1 = 8;  // Left motor direction 1
int inL2 = 9;  // Left motor direction 2
int enR = 12;  // Right motor PWM
int inR1 = 10; // Right motor direction 1
int inR2 = 11; // Right motor direction 2

// For encoder
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;

// Target encoder counts
long inner_target = 15000; // Target encoder counts for the inner wheel
long outer_target = 18000; // Target encoder counts for the outer wheel

// Robot parameters
float wheelbase = 0.189; // Wheelbase of the robot in meters
float radius = 1.0; // Radius of the circular path in meters
float baseSpeed = 150; // Speed of the robot’s center in PWM
const int K = 30; // Proportional gain

// Calculated speeds
int leftMotorSpeed;
int rightMotorSpeed;
volatile long rightEnCount = 0;
volatile long leftEnCount = 0;

long tmpleft = 0;
long tmpright = 0;
void setup() {
  Serial.begin(9600);

  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);
  
  // Initialize motor control pins
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
}

void loop() {
  tmpleft = tmpleft + leftEnCount;
  tmpright = tmpright + rightEnCount;
  rightEnCount = 0;
  leftEnCount = 0;
  
  // Calculate wheel speeds for a circular path
  leftMotorSpeed = 180; // Example value; adjust as needed

  // Adjust right motor speed
  const float turnWeight = 0.7;
  rightMotorSpeed = turnWeight * leftMotorSpeed + K * (turnWeight * rightEnCount - leftEnCount);

  // Ensure PWM values are within range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Set motor speeds
  analogWrite(enL, leftMotorSpeed);
  analogWrite(enR, rightMotorSpeed);

  // Set initial motor directions
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  // Print wheel speeds
  Serial.print("V right: ");
  Serial.println(tmpleft); 
  Serial.print("V left: ");
  Serial.println(tmpright);

  // Check if the target encoder counts are reached
  if (tmpright >= inner_target || tmpleft >= outer_target) {
    stop();
    while (true) {
    }
  }
  
  delay(100); 
}

void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

void stop() {
  // Turn off motors 
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}
