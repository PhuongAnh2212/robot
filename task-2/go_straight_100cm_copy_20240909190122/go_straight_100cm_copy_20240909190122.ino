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
// const int pulsesPerRevolution = 350 * 2 / 3; // Encoder pulses per revolution of the motor
// const int targetPulses = 4700;  // Target pulses to stop the motors
const float wheelbase = 0.189;  // Distance between the wheels in meters
const float radius = 1.0;  // Radius of the circle (1 meter)
unsigned long lastTime = 0; // To track time for RPM calculation
const int baseSpeed = 128; // Base PWM speed

const unsigned long interval = 1000; // Interval in ms for calculating RPM

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
    rpm1 = leftEnCount / 2.0;  // Example formula
    rpm2 = rightEnCount / 2.0;

    // Print RPM values
    Serial.print("Left Motor RPM: ");
    Serial.println(rpm1);
    Serial.print("Right Motor RPM: ");
    Serial.println(rpm2);
  }

  // Move in a circle
  moveInCircle(baseSpeed);
}

void moveInCircle(int speed) {
  // Calculate the speeds needed for the left and right motors
  float innerRadius = radius - (wheelbase / 2);
  float outerRadius = radius + (wheelbase / 2);

  // Set speed proportionally
  int leftSpeed = speed * (innerRadius / radius);
  int rightSpeed = speed * (outerRadius / radius);

  // Constrain motor speed to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Set motor speeds
  analogWrite(enL, leftSpeed);
  analogWrite(enR, rightSpeed);

  // Turn on motors to go forward
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
void leftEnISR() {
  leftEnCount++;
}

void rightEnISR() {
  rightEnCount++;
}