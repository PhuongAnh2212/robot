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

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float rpm1 = 0;
float rpm2 = 0;

const int K = 30;  //adjust K for smooth response
const int pulsesPerRevolution = 350 * 2 / 3; // Encoder pulses per revolution of the motor
// const int targetPulses = 4700;  // Target pulses to stop the motors
const float wheelbase = 0.189; // Example value in meters (distance between the two wheels)
const float radius = 1.0;    // Circle radius in meters (half of 2 meters)
const int baseSpeed = 128;   // Base motor speed (adjust as necessary)
unsigned long lastTime = 0; // To track time for RPM calculation
const unsigned long interval = 1000; // Interval in ms for calculating RPM

float R_inner = radius - (wheelbase / 2);
float R_outer = radius + (wheelbase / 2);

float speedRatio = R_outer / R_inner;


void setup()
{
  Serial.begin(9600);

  // Setup interrupt 
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
  // Every 1 second, calculate and print RPM
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    
    // Calculate RPM for left and right motors
    rpm1 = (leftEnCount / (float)pulsesPerRevolution) * 60;  // RPM formula
    rpm2 = (rightEnCount / (float)pulsesPerRevolution) * 60;
    
    // Print RPM values
    Serial.print("Left Motor RPM: ");
    Serial.println(rpm1);
    Serial.print("Right Motor RPM: ");
    Serial.println(rpm2);
    
    // Print pulse counts
    Serial.print("Left Motor Pulse Count: ");
    Serial.println(leftEnCount);
    Serial.print("Right Motor Pulse Count: ");
    Serial.println(rightEnCount);
  }

  // Stop the motors if either encoder reaches the target pulse count
  // if (leftEnCount >= targetPulses || rightEnCount >= targetPulses) {
  //   stop();  // Stop both motors
  //   Serial.println("Target reached. Motors stopped.");
  //   while (true);  // Stop the program here
  // }

//   goForward(128); // Call to goForward function with base speed
// }
  moveInCircle(baseSpeed);
}

// void goForward(int speed) {
//   // Adjust motor speeds with K-factor to compensate for differences
//   int motor_R_speed = speed + K * (leftEnCount - rightEnCount);  

//   // Constrain motor speed to valid PWM range (0-255)
//   motor_R_speed = constrain(motor_R_speed, 0, 255);
  
//   // Set motor speeds
//   analogWrite(enL, speed);
//   analogWrite(enR, motor_R_speed);

//   // Turn on motors to go forward
//   digitalWrite(inL1, HIGH);
//   digitalWrite(inL2, LOW);
//   digitalWrite(inR1, LOW);
//   digitalWrite(inR2, HIGH);
// }

void moveInCircle(int speed) {
  // Adjust motor speeds for circular motion
  int leftSpeed = speed;  // Inner wheel
  int motor_R_speed = speed + K * speedRatio;  

  // Set motor speeds
  analogWrite(enL, leftSpeed);
  analogWrite(enR, motor_R_speed);

  // Turn on motors to move forward in a circle
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
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

void leftEnISRB() {
  leftEnCount++;
}
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}