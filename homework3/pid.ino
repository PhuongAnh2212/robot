// Motor control pins
const int enL = 7;   // PWM pin for left motor
const int inL1 = 8;  // Motor direction pin 1 for left motor
const int inL2 = 9;  // Motor direction pin 2 for left motor
const int enR = 12;  // PWM pin for right motor
const int inR1 = 10; // Motor direction pin 1 for right motor
const int inR2 = 11; // Motor direction pin 2 for right motor

// Encoder pins
const int enLA = 2;  // Encoder A pin for left motor
const int enLB = 3;  // Encoder B pin for left motor
const int enRA = 18; // Encoder A pin for right motor
const int enRB = 19; // Encoder B pin for right motor

// Control constants
const float d = 0.189;    // Distance between wheels (meters)
const float r = 0.0225;   // Radius of the wheels (meters)
const float T = 1.2;      // Time step (seconds)
const float gamma = 0.26; // Linear control gain
const float lamda = 0.32; // Angular control gain
const float h = 0.21;     // Offset in angle calculation
const float goalX = 1.0;  // Goal X position (meters)
const float goalY = 1.0;  // Goal Y position (meters)
const float goalTheta = 0.0; // Goal orientation (radians)

// Encoder constants
const int encoderCountsPerRevolution = 350; // 1:50 encoder with 7 pulses per revolution
const float wheelDiameter = 0.0225 * 2; // Diameter of the wheel in meters
const float wheelCircumference = wheelDiameter * M_PI; // Wheel circumference

// PID control constants
float Kp_linear = 2.9, Ki_linear = 0.2001, Kd_linear = 0.07;
float Kp_angular = 35.0, Ki_angular = 0.1, Kd_angular = 0.05;

// PID variables
float previousErrorLinear = 0.0, integralLinear = 0.0;
float previousErrorAngular = 0.0, integralAngular = 0.0;

// Robot state variables
float x = 0.0;     // Current X position (meters)
float y = 0.0;     // Current Y position (meters)
float theta = 0.0; // Current orientation (radians)

// Encoder counts and speed control
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
const int K = 30;  // Adjustment factor for speed control

void setup() {
  Serial.begin(9600);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop() {
  // Calculate errors
  float deltaX = goalX - x;
  float deltaY = goalY - y;
  float rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  float alpha = atan2(deltaY, deltaX) - theta;

  // PID control for linear velocity
  float errorLinear = rho;
  integralLinear += errorLinear * T;
  float derivativeLinear = (errorLinear - previousErrorLinear) / T;
  float v = Kp_linear * errorLinear + Ki_linear * integralLinear + Kd_linear * derivativeLinear;
  previousErrorLinear = errorLinear;

  // PID control for angular velocity
  float errorAngular = alpha;
  integralAngular += errorAngular * T;
  float derivativeAngular = (errorAngular - previousErrorAngular) / T;
  float w = Kp_angular * errorAngular + Ki_angular * integralAngular + Kd_angular * derivativeAngular;
  previousErrorAngular = errorAngular;

  // Wheel velocities
  float vr = v + d * w / 2.0;
  float vl = v - d * w / 2.0;
  float wr = vr * 60.0 / (2.0 * M_PI * r); // Angular velocity in RPM
  float wl = vl * 60.0 / (2.0 * M_PI * r);

  // Set wheel speeds
  set_speedL(wl);
  set_speedR(wr);

  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);

  // Wait for the time step
  delay(T * 1000); // Convert seconds to milliseconds

  // Update position
  float v1 = get_speedL();
  float v2 = get_speedR();
  x += (v1 + v2) / 2.0 * cos(theta) * T;
  y += (v1 + v2) / 2.0 * sin(theta) * T;
  theta += (v2 - v1) / d * T;

  // Print debugging information
  // Serial.print("X: ");
  // Serial.print(x);
  // Serial.print(" Y: ");
  // Serial.print(y);
  // Serial.print(" Theta: ");
  // Serial.print(theta);
  // Serial.print(" Left Speed: ");
  // Serial.print(v1);
  // Serial.print(" Right Speed: ");
  // Serial.println(v2);

  // Check if goal is reached
  if (fabs(goalX - x) <= 0.51 && fabs(goalY - y) <= 0.51) {
    stop();
    // Serial.println("Goal reached!");
    while (1); // Stop further execution
  }
}

// Function to set the speed of the left wheel
void set_speedL(float speed) {
  int pwmValue = constrain(speed, 0, 70); // Constrain speed to PWM range
  analogWrite(enL, pwmValue);
}

// Function to set the speed of the right wheel
void set_speedR(float speed) {
  int pwmValue = constrain(speed, 0, 70); // Constrain speed to PWM range
  analogWrite(enR, pwmValue);
}

// Function to get the current speed of the left wheel
float get_speedL() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = leftEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / encoderCountsPerRevolution; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  leftEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

// Function to get the current speed of the right wheel
float get_speedR() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = rightEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / encoderCountsPerRevolution; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  rightEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

// Encoder ISR for left wheel
void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

// Encoder ISR for right wheel
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

// Stop the motors
void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}
