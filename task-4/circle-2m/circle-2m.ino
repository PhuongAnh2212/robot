// Left Motor
int enL = 7;
int inL1 = 8;
int inL2 = 9;

// Right motor
int enR = 12;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 18;
int enRB = 19;

float rpm1 = 0;
float rpm2 = 0;

const int K = 30;  // Adjust K for smooth response
const float radius = 1.0;    // Circle radius in meters (half of 2 meters)
unsigned long lastTime = 0;  // To track time for RPM calculation
const unsigned long interval = 1000; // Interval in ms for calculating RPM

// PID control parameters
float Kp = 1.5;   // Proportional gain
float Ki = 0.01;  // Integral gain
float Kd = 0.05;  // Derivative gain

// Variables for PID control
float previousErrorL = 0, previousErrorR = 0;
float integralL = 0, integralR = 0;

// For encoder and RPM calculation
const int pulsesPerRevolution = 350 * 2 / 3;
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float rpmL = 0, rpmR = 0;


void setup() {
  Serial.begin(9600);
  
  // Setup interrupt for encoders
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);

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
  // Calculate RPM every 1 second
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Calculate RPM from encoder counts
    rpmL = (leftEnCount / (float)pulsesPerRevolution) * 60;
    rpmR = (rightEnCount / (float)pulsesPerRevolution) * 60;

    // Reset encoder counts for next interval
    leftEnCount = 0;
    rightEnCount = 0;

    // Print RPM values for debugging
    Serial.print("Left Motor RPM: ");
    Serial.println(rpmL);
    Serial.print("Right Motor RPM: ");
    Serial.println(rpmR);

    int targetSpeedL = 181;
    int targetSpeedR = 150;

    // Apply PID control to both wheels
    int motorL_speed = pidControl(targetSpeedL, rpmL, &previousErrorL, &integralL);
    int motorR_speed = pidControl(targetSpeedR, rpmR, &previousErrorR, &integralR);

    // Set the motor speeds
    analogWrite(enL, motorL_speed);
    analogWrite(enR, motorR_speed);
  }
}

// PID control function
int pidControl(float targetSpeed, float currentRPM, float* previousError, float* integral) {
  // Calculate error
  float error = targetSpeed - currentRPM;

  // Calculate integral
  *integral += error;

  // Calculate derivative
  float derivative = error - *previousError;

  // Update previous error
  *previousError = error;

  // Calculate the control output
  float output = Kp * error + Ki * (*integral) + Kd * derivative;

  // Constrain the output to valid PWM range (0-255)
  int motorSpeed = constrain(output, 0, 255);

  return motorSpeed;
}

void stop() {
  // Turn off motors
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

// Interrupt Service Routines (ISRs) for counting encoder pulses
void leftEnISRA() {
  leftEnCount++;
}

void rightEnISRA() {
  rightEnCount++;
}
