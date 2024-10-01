// Left Motor
int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the motor 
int inL2 = 9;

// Right Motor
int enR = 12; 
int inR1 = 10;
int inR2 = 11;

// For Encoder
int enLA = 2;  // read one of the signals from the left motor
int enLB = 3;

int enRA = 18; // read one of the signals from the right motor 
int enRB = 19;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0; 
const int targetSpeed = 50; 
const int K = 3;  // Adjust K for smooth response

#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4 

void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void goforward() {
  // Calculate left motor speed based on encoder counts
  int motor_L_speed = targetSpeed + K * (rightEncoderCount - leftEncoderCount);
  int motor_R_speed = targetSpeed;  // Keep the right motor speed as targetSpeed

  // Write PWM values ensuring they are within valid range
  analogWrite(enL, constrain(motor_L_speed, 0, 255));
  analogWrite(enR, constrain(motor_R_speed, 0, 255));

  // Set motor directions
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnLeft() {
  // Reset encoder counter
  rightEncoderCount = 0;
  leftEncoderCount = 0;

  int speed = 50; // Set speed for turning

  // Set left motor to stop and right motor to move forward
  analogWrite(enL, 0); // Stop left motor
  analogWrite(enR, speed); // Keep right motor moving forward

  // Set motor directions
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);  
}

void turnRight() {
  // Reset encoder counter
  rightEncoderCount = 0;
  leftEncoderCount = 0;

  int speed = 50; // Set speed for turning

  // Set right motor to stop and left motor to move forward
  analogWrite(enR, 0); // Stop right motor
  analogWrite(enL, speed); // Keep left motor moving forward

  // Set motor directions
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);  
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void setup() {
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT); 
  attachInterrupt(digitalPinToInterrupt(enLA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEncoderISR, RISING);
}

void loop() {
  // Read sensor values
  int s1 = digitalRead(ir1); // left most sensor 
  int s2 = digitalRead(ir2); // left sensor
  int s3 = digitalRead(ir3); // middle sensor
  int s4 = digitalRead(ir4); // right sensor
  int s5 = digitalRead(ir5); // right most sensor 

  // Go forward if only the middle sensor detects white line
  if ( 
      (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) ||
      (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0) ||
      (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0)) 
  
      {
    goforward();
  } 
  // Turn left if the two leftmost sensors detect white
  else if ((s1 == 1 && s2 == 0) || (s1 == 1 && s2 == 1)||
  (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0)){
    turnRight();
  } 
  // Turn right if the two right sensors detect white
  else if ((s5 == 1 && s4 == 0) || (s5 == 1 && s4 == 1)||
  (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0)||
  (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0)) {
    turnLeft();
  } 
  // Stop if no conditions are met
  else {
    stop();
  }

  // Reset encoder counts for the next loop iteration
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}
