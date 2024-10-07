// Motor control pins
int enL = 7; 
int inL1 = 8; 
int inL2 = 9;

int enR = 12; 
int inR1 = 10;
int inR2 = 11;

// Ultrasonic sensor pins
const int frontTrigPin = 53;
const int frontEchoPin = 52;
const int leftTrigPin = 49;
const int leftEchoPin = 48;
const int rightTrigPin = 51;
const int rightEchoPin = 50;

// PID variables
float Kp = 4.5, Ki = 0.02, Kd = 0.5;
const int initial_motor_speed = 55; 
const int thresholdDistance = 40; 
float sampling_rate = 5; 
float previousTime;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;

int sensor[5] = {0, 0, 0, 0, 0};

//Ultrasonic

// Encoder counts and speed control
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
const int K = 30;

void setup() {
    Serial.begin(9600);
    pinMode(enL, OUTPUT);
    pinMode(enR, OUTPUT);
    pinMode(inL1, OUTPUT);
    pinMode(inL2, OUTPUT);
    pinMode(inR1, OUTPUT);
    pinMode(inR2, OUTPUT);
    pinMode(frontTrigPin, OUTPUT);
    pinMode(frontEchoPin, INPUT);
    pinMode(leftTrigPin, OUTPUT);
    pinMode(leftEchoPin, INPUT);
    pinMode(rightTrigPin, OUTPUT);
    pinMode(rightEchoPin, INPUT);

    // Initialize motors to move forward
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
    Serial.println("Motors initialized.");
}

void loop() {
    long frontDistance = measureDistance(frontTrigPin, frontEchoPin);
    long leftDistance = measureDistance(leftTrigPin, leftEchoPin);
    long rightDistance = measureDistance(rightTrigPin, rightEchoPin);
    
    Serial.print("Front: "); Serial.print(frontDistance);
    Serial.print(" | Left: "); Serial.print(leftDistance);
    Serial.print(" | Right: "); Serial.println(rightDistance);

 // Obstacle avoidance logic
  if (frontDistance > thresholdDistance && leftDistance > thresholdDistance && rightDistance > thresholdDistance) {
    // Move forward if no obstacle is detected in the front or left
    moveForward();
  } 
  else if (frontDistance <= thresholdDistance) {
    // Obstacle in front, avoid by turning right
    stop();
    delay(500);
    
    // Move backward for a moment
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
    delay(500);

    // Turn right
    stop();
    delay(500);
    turnLeft();
    delay(500);  
  } 
  else if (leftDistance <= thresholdDistance) {
    // Obstacle on the left, turn right
    stop();
    delay(500);
    
    // Turn right to avoid the left obstacle
    turnRight();
    delay(500);
  }

  else if (rightDistance <= thresholdDistance) {
    // Obstacle on the left, turn right
    stop();
    delay(500);
    
    // Turn left to avoid the left obstacle
    turnLeft();
    delay(500);
  }
    //Obstacle ở bên phải và trước mặt  
    else if (rightDistance <= thresholdDistance && frontDistance <= thresholdDistance) {
    stop();
    delay(500);
    
    // Turn left to avoid the obstacle
    turnLeft();
    delay(500);
    }

    //Obstacle ở bên trái và trước mặt
    else if (leftDistance <= thresholdDistance && frontDistance <= thresholdDistance) {
    stop();
    delay(500);
    
    // Turn left to avoid the obstacle
    turnRight();
    delay(500);
    }

    //Obstacle ở bên trái và bên phải => đi thẳng
    else if (leftDistance <= thresholdDistance && rightDistance <= thresholdDistance) {
    stop();
    delay(500);
    
    moveForward();
    delay(500);
    }

    else if (leftDistance <= thresholdDistance && rightDistance <= thresholdDistance && frontDistance <= thresholdDistance) {
    stop();
    delay(500);
    
    moveBackward();
    delay(500);
    }

    
  // Add a short delay to stabilize readings
  delay(100);
}

void avoidObstacleFront() {
  // Stop the motors
  stop();
  delay(500);  // Pause for a moment

  // Move backward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
  delay(500);

    // Stop and turn right
  stop();
  delay(500);
  turnRight();
  delay(500);
}

void stop(){
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void moveBackward(){
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);

  motor_control();

}

// Function to turn the robot to the right
void turnRight() {
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = initial_motor_speed;
  const int turnWeight = 1.8;
  analogWrite(enR, speed);

  int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

// Function to turn the robot to the right
void turnLeft() {
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = initial_motor_speed;
  const int turnWeight = 1.5;
  analogWrite(enL, speed);

  int motor_R_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);  
  analogWrite(enR, motor_R_speed);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

long measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2; // Convert to centimeters
}

void motor_control() {

  // Calculate the effective motor speed based on PID
  int left_motor_speed = initial_motor_speed;
  int right_motor_speed = initial_motor_speed;

  // Constrain motor speeds
  left_motor_speed = constrain(left_motor_speed, 0, 150);
  right_motor_speed = constrain(right_motor_speed, 0, 150);

  analogWrite(enL, left_motor_speed);  // Set left motor speed
  analogWrite(enR, right_motor_speed); // Set right motor speed
}

void moveForward() {
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  motor_control();

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
