// Define motor control pins
int enL = 7;   // Left motor PWM
int inL1 = 8;  // Left motor direction 1
int inL2 = 9;  // Left motor direction 2
int enR = 12;  // Right motor PWM
int inR1 = 10; // Right motor direction 1
int inR2 = 11; // Right motor direction 2

// Encoder pins
int enLA = 2;  // Left encoder A (interrupt pin)
int enLB = 3;  // Left encoder B (if used, not in this example)
int enRA = 18; // Right encoder A (interrupt pin)
int enRB = 19; // Right encoder B (if used, not in this example)

const int K = 30;  //adjust K for smooth response

// Pulse counters
volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;

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
  
  
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach interrupts for encoder pulses
  attachInterrupt(digitalPinToInterrupt(enLA), countLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countRightPulses, RISING);
}

void loop() {
    // Set motor speeds
  analogWrite(enL, 150);
  analogWrite(enR, 181.3);
  // Print the pulse counts every second
  Serial.print("Left Pulses: ");
  Serial.print(leftPulseCount);
  Serial.print(" | Right Pulses: ");
  Serial.println(rightPulseCount);
  
  // Add a delay to avoid overwhelming the serial monitor
  delay(1000); // Delay 1 second
}

void countLeftPulses() {
  // Increment the left pulse counter
  leftPulseCount++;
}

void countRightPulses() {
  // Increment the right pulse counter
  rightPulseCount++;
}
