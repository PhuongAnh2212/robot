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
long inner_target = 9100;
long outer_target = 11000;

// Robot parameters
float wheelbase = 0.189; // Wheelbase of the robot in meters
float radius = 1.0; // Radius of the circular path in meters
float baseSpeed = 150; // Speed of the robot’s center in PWM
const int K = 30;
// Calculated speeds
int leftMotorSpeed;
int rightMotorSpeed;
volatile long rightEnCount = 0;
volatile long leftEnCount = 0; 


void setup() {

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
  
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  rightEnCount = 0;
  leftEnCount = 0;

  
   // Calculate wheel speeds for a circular path
  float V_center = baseSpeed; // Center speed for the robot
  //leftMotorSpeed = V_center * (1 + wheelbase / (2 * radius));
  //rightMotorSpeed = V_center * (1 - wheelbase / (2 * radius));
  //rightMotorSpeed = V_center * ((radius - (wheelbase / 2))/ radius);
  //leftMotorSpeed = V_center * radius * (1 + (wheelbase / 2));

  leftMotorSpeed = 180;

  const float turnWeight = 0.7;
   Serial.println(leftEnCount);
  Serial.println(rightEnCount);
  rightMotorSpeed = turnWeight*leftMotorSpeed + K*(turnWeight*rightEnCount-leftEnCount);  
  //int motor_R_speed = turnWeight*baseSpeed + K*(turnWeight*leftEnCount-rightEnCount);  
  //analogWrite(enR, motor_R_speed);
  
  // Ensure PWM values are within range
  //leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  //rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

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
  Serial.println(rightMotorSpeed); 
  Serial.print("V left: ");
  Serial.println(leftMotorSpeed);

  delay(1000);
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