// Motor pins
int enL = 7;   // Left motor PWM
int inL1 = 8;  // Left motor direction 1
int inL2 = 9;  // Left motor direction 2
int enR = 12;  // Right motor PWM
int inR1 = 10; // Right motor direction 1
int inR2 = 11; // Right motor direction 2

// Encoder pins
int enLA = 2;  // Left encoder A (interrupt pin)
int enRA = 18; // Right encoder A (interrupt pin)

// Variables to store pulse counts
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;

// Interrupt service routines for counting encoder pulses
void countLeftPulse() {
  leftPulseCount++;
}

void countRightPulse() {
  rightPulseCount++;
}

void setup() {
  // Set motor pins as outputs
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  
  // Set encoder pins as inputs
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);
  
  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(enLA), countLeftPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countRightPulse, RISING);

  Serial.begin(9600);
}

void loop() {
  // Reset pulse counters
  leftPulseCount = 0;
  rightPulseCount = 0;

  // Set motors to run forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);

  // Set motor speeds (adjust as needed)
  analogWrite(enL, 255); // Max speed for left motor
  analogWrite(enR, 255); // Max speed for right motor

  // Run motors for 1 second
  delay(1000);

  // Stop motors
  analogWrite(enL, 0);
  analogWrite(enR, 0);

  // Print pulse counts for both motors
  Serial.print("Left Motor Pulse Count: ");
  Serial.println(leftPulseCount);
  Serial.print("Right Motor Pulse Count: ");
  Serial.println(rightPulseCount);

  // Wait before next run
  delay(1000);
}
