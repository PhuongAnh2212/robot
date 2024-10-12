#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <HCSR04.h>


bool flag = true;

// Ultrasonic sensor pins
const int frontTrigPin = 53;
const int frontEchoPin = 52;
const int leftTrigPin = 49;
const int leftEchoPin = 48;

HCSR04 frontSonar(53, 52); //initialisation class HCSR04 (trig pin , echo pin)
HCSR04 leftSonar(49, 48); //initialisation class HCSR04 (trig pin , echo pin)
HCSR04 rightSonar(51, 50); //initialisation class HCSR04 (trig pin , echo pin)

float previousTime_sona = 0.0;
float sampling_rate_sona = 50; //don vi la miliseconds

const int obstacleThreshold = 20; // cm
const int safeDistance = 10;

volatile long leftEnCount = 0;
volatile long rightEnCount = 0;


float Kp_sona = 0.6, Ki_sona = 0.8, Kd_sona = 3.0;

// Variables for PID controller
float P_sona = 0, I_sona = 0, D_sona = 0;
float previous_error_sona = 0;
float PID_value_sona = 0;

//_______________________________
// Motor control pins
int enL = 7; // enable the left motor driver, using PWM
int inL1 = 8; // control direction of the left motor
int inL2 = 9;

#define echo 51    //Echo pin
#define trigger 50 //Trigger pin
// Right Motor
int enR = 12;
int inR1 = 10;
int inR2 = 11;

// IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
float currentAngle = 0.0;

// For Encoder
int enLA = 2;  // left motor encoder
int enLB = 3;
int enRA = 18; // right motor encoder
int enRB = 19;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
//const int targetSpeed = 40;
const int K = 3;  // Adjust K for smooth response

// PID variables
float Kp = 3, Ki = 0.02, Kd = 0.5;
float sampling_rate = 0;
float previousTime;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;

// Sensor array and motor speed
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 60;

// Function prototypes
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void setup();
void loop();

void setup() {

  Serial.begin(9600);
  // Initialize IMU
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  previousTime = 0;
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  // Initialize motor direction (forward)
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  // Set ultrasonic pins
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  // Set encoder pins
  pinMode(enLA, INPUT);
  pinMode(enLB, INPUT);
  pinMode(enRA, INPUT);
  pinMode(enRB, INPUT);


  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);
}

//_____________________________________
enum RobotState {
  LineFollower,
  AvoidObstacle,
  BacktotheLine
};

RobotState currentState = LineFollower;



void loop() {

  switch (currentState) {
    case LineFollower:
      checkPIDState();
      Serial.println("PID IRs State");
      break;


    case AvoidObstacle:
      checkAvoidObstacleState();
      Serial.println("Obstacle State");
      break;


    case BacktotheLine:
      checkLineStatus();
      Serial.println("Back to the Line State");
  }

}

//*********************************************
void checkLineStatus() {
  if (lineDetectedbyIRs()) {
    stop();
    delay(1000);
    Serial.println("checkLineStatus");
    currentState = LineFollower;

  }
}
bool lineDetectedbyIRs() {
  for (int i = 0; i < 5; i++) {
    sensor[i] = digitalRead(A0 + i);
    //    if (sensor[0]==1 || sensor[1]==1 || sensor[2]==1 || sensor[3]==1 || sensor[4]==1 ){
    if (sensor[i] == 1 ) {
      Serial.print("let there be light");
      return true;
    }
  }
  return false;

}

int calculateTargetCounts(float distance_en) {
  const int encoderCountsPerRevolution = 350; // 1:50 encoder with 7 pulses per revolution
  const float wheelDiameter = 0.0225 * 2; // Diameter of the wheel in meters
  const float wheelCircumference = wheelDiameter * 3.14; // Wheel circumference

  return (distance_en / wheelCircumference) * encoderCountsPerRevolution;


}

void moveForward_encoder(float distance_en) {
  int targetCounts = calculateTargetCounts(distance_en);
  leftEnCount = 0;
  rightEnCount = 0;


  Serial.println("start moving");
  // Wait until the robot has moved the target distance
  while (leftEnCount <= targetCounts && rightEnCount <= targetCounts) {
    // Set direction for moving forward
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);

    // Start moving forward
    analogWrite(enL, 70);  // Use initial speed
    analogWrite(enR, 73);  // Use initial speed
  }
  Serial.println("stop moving");
  stop();
}

void moveForward_encoder_final(float distance_en) {
  int targetCounts = calculateTargetCounts(distance_en);
  leftEnCount = 0;
  rightEnCount = 0;


  Serial.println("start moving");
  // Wait until the robot has moved the target distance
  while (leftEnCount <= targetCounts && rightEnCount <= targetCounts) {
    // Set direction for moving forward
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);

    // Start moving forward
    analogWrite(enL, 92);  // Use initial speed
    analogWrite(enR, 97);  // Use initial speed
  }
  Serial.println("stop moving");
  stop();
}


void moveForward_encoder_final1(float distance_en) {
  int targetCounts = calculateTargetCounts(distance_en);
  leftEnCount = 0;
  rightEnCount = 0;


  Serial.println("start moving");
  // Wait until the robot has moved the target distance
  while (leftEnCount <= targetCounts && rightEnCount <= targetCounts) {
    // Set direction for moving forward
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);

    // Start moving forward
    analogWrite(enL, 90);  // Use initial speed
    analogWrite(enR, 95);  // Use initial speed
  }
  Serial.println("stop moving");
  stop();
}



void moveBackward_encoder(float distance_en) {
  int targetCounts = calculateTargetCounts(distance_en);
  leftEnCount = 0;
  rightEnCount = 0;


  Serial.println("start moving");
  // Wait until the robot has moved the target distance
  while (leftEnCount <= targetCounts && rightEnCount <= targetCounts) {
    // Set direction for moving forward
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);

    // Start moving forward
    analogWrite(enL, 50);  // Use initial speed
    analogWrite(enR, 50);  // Use initial speed
  }
  Serial.println("stop moving");
  stop();
}



void hardcode_turnAround() {
  analogWrite(enR, 80);
  analogWrite(enL, 80);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);

  //  if (lineDetectedbyIRs() && isFirstTimeDetected) {
  //    stop();
  //    delay(3000);
  //    isFirstTimeDetected = false;
  //    return;
  //  }
}



void checkAvoidObstacleState() {

  Serial.print("nao thay obs roi");
  // 1
  stop();
  delay(2000);

  turnRightDegrees(35);
  stop();
  Serial.println("meofmeosmeo");
  delay(1000);


  moveForward_encoder(0.6);
  delay(300);

  //2
  turnLeftDegrees(10);

  Serial.println("escape left then read sona");
  delay(300);

//  moveForward_encoder(2.0);
//
//  // 3
//  turnLeftDegrees(10);
//
//  travelUntilSeeLight();
  Serial.println("see light trong vong loop obstacle");
  
  currentState = LineFollower;  // Return to normal state if no obstacles are detected
}

void checkPIDState() {
  Serial.println("Loop is running."); // This will print repeatedly

  // Move forward at a specified speed
  read_sensor_values();
  motor_control();

  // Check for obstacles in front
  if (ObstacleFront()) {
    Serial.println("Obstacle detected");
    currentState = AvoidObstacle;  // Transition to avoid obstacle state
  }
}


void hardcode_turnRight() {
  analogWrite(enR, 50);
  analogWrite(enL, 90);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  Serial.print("Turn Right 90 degrees hardcode");

}



bool ObstacleFront() {
  long distanceFront = get_front_distance();
  return distanceFront < obstacleThreshold; // Return true if obstacle is detected
}



float get_front_distance() {
  float distance = frontSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;

  }
  return distance;

}

float get_left_distance() {
  float distance = leftSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;

  }
  return distance;

}


void read_sona_values() {
  //  float currentTime_sona = millis();

  Serial.println("doc gia tri sona");
  // Get distance values from left and right sonar sensors
  long distanceLeft = get_left_distance();
  //  long distanceRight = get_right_distance();

  Serial.println("Distance Left:");
  Serial.println(distanceLeft);

  // Check if there are obstacles and calculate PID if needed
  if (distanceLeft < safeDistance) {
    float error_sona = safeDistance - distanceLeft - 1; // Error = difference in distances
    float pid = calculate_pid_sona(error_sona); // Call the PID controller with the calculated error
    motor_control_sona(PID_value_sona);
    Serial.println("meowssssss");

  }
  else if (distanceLeft >= safeDistance) {
    float error_sona = distanceLeft - safeDistance + 1; // Error = difference in distances
    float pid = calculate_pid_sona(error_sona); // Call the PID controller with the calculated error
    motor_control_sona(pid);
    Serial.println("ssssssswoem");

  }
}

void motor_control_sona(float PID_value_sona) {
  // Calculating the effective motor speed:
  int left_motor_speed = abs(initial_motor_speed - (PID_value_sona / 100));
  int right_motor_speed = abs(initial_motor_speed + (PID_value_sona / 100));

  left_motor_speed = constrain(left_motor_speed, 0, 60);
  right_motor_speed = constrain(right_motor_speed, 0, 60);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);


  analogWrite(enL, left_motor_speed);  // Set left motor speed
  analogWrite(enR, right_motor_speed); // Set right motor speed

  Serial.println("left:");
  Serial.println(left_motor_speed);
  Serial.println("right");
  Serial.print(right_motor_speed);

}

float calculate_pid_sona(float error_sona) {
  P_sona = error_sona;
  I_sona += error_sona;
  D_sona = error_sona - previous_error_sona;

  PID_value_sona = (Kp_sona * P_sona) + (Ki_sona * I_sona) + (Kd_sona * D_sona);
  previous_error_sona = error_sona;
  Serial.println("PID value:");
  Serial.println(PID_value_sona);


  return PID_value_sona;
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  analogWrite(enL, 0);  // Set left motor speed
  analogWrite(enR, 0); // Set right motor speed
}


//__________________________
void read_sensor_values() {
  Serial.print("doc IR");
  float currentTime = millis();
  if (currentTime - previousTime >= sampling_rate) {
    for (int i = 0; i < 5; i++) {
      sensor[i] = digitalRead(A0 + i); // Assume sensors are connected to A0 to A4
    }
    //phai sang, 1 den don sang
    if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) { //00100
      error = 0;
      calculate_pid(error);
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)) { //00010
      error = 2;
      calculate_pid(error);
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) { //00001
      error = 5;
      calculate_pid(error);
    }
    //phai sang, truong hop 2 den sang
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) { //00110
      error = 2;
      calculate_pid(error);
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) { //00011
      error = 3;
      calculate_pid(error);
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) { //00101
      error = 3;
      calculate_pid(error);
    }

    //trai sang, 1 den don sang
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //01000
      error = -2;
      calculate_pid(error);
    }
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //10000
      error = -5;
      calculate_pid(error);
    }

    //trai sang, 2 den don sang
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) { //01100
      error = -1;
      calculate_pid(error);
    }
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //11000
      error = -3;
      calculate_pid(error);
    }
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) { //10100
      error = -3;
      calculate_pid(error);
    }

    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) { //11111
      stop();
      delay(1000);
      moveForward_encoder_final1(1.4);
      stop(); 
      delay(1000);
      turnLeftDegrees(60);
      moveForward_encoder_final(6.8);
    }

    previousTime = currentTime;
  }



  //    if ((error == -4) || (error == -5)) {
  //      error = -5;
  //      calculate_pid(error);
  //      //count = count + 1;
  //    } else if ((error == 4) || (error == 5)) {
  //      error = 5;
  //      calculate_pid(error);
  // count = count + 1;
  //    }
  //  }

}

void calculate_pid(float error) {
  P = error;
  I += error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}

void motor_control() {

  // Calculate the effective motor speed based on PID
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // Constrain motor speeds
  left_motor_speed = constrain(left_motor_speed, 0, 150);
  right_motor_speed = constrain(right_motor_speed, 0, 150);

  analogWrite(enL, left_motor_speed);  // Set left motor speed
  analogWrite(enR, right_motor_speed); // Set right motor speed

  Serial.print("Left Speed: "); Serial.print(left_motor_speed);
  Serial.print(" | Right Speed: "); Serial.println(right_motor_speed);
}

void pid_line_follow() {
  read_sensor_values();
  motor_control();
}


void turnRight(int speed) {
  leftEnCount = 0;
  rightEnCount = 0;
  const int turnWeight = 2;
  analogWrite(enR, speed);

  int motor_L_speed = turnWeight * speed + K * (turnWeight * rightEnCount - leftEnCount);
  motor_L_speed = constrain(motor_L_speed, 0, 255); // Ensure PWM is in valid range
  analogWrite(enL, motor_L_speed);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnRightDegrees(float degrees) {
  // Reset encoder counters

  bno.begin();
  delay(1000);
  // Get the starting angle
  float startAngle = getCurrentYaw();
  Serial.println("start angle");

  Serial.println(startAngle);
  float targetAngle = startAngle + degrees; // Target angle is current + desired degrees
  //if targetAngle > 360 => then minus 360
  if (targetAngle > 360) {
    targetAngle = targetAngle - 360;
    while ((getCurrentYaw() >= startAngle && getCurrentYaw() <= 360)  || (getCurrentYaw() >= 0 && getCurrentYaw() < targetAngle) ) {
      turnRight(60);
      Serial.print("right lan 1 ");
      Serial.print("Current Yaw Angle: ");
      Serial.println(getCurrentYaw());
      delay(500);
    }

  } else {

    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() < targetAngle) {
      // Print the current yaw angle
      Serial.print("Current Yaw Angle: ");
      Serial.println(getCurrentYaw());
      turnRight(60);
      Serial.print("right lan 2 ");
    }
  }

}

void turnLeft(int speed) {
  leftEnCount = 0;
  rightEnCount = 0;
  const int turnWeight = 2;
  analogWrite(enL, speed);

  int motor_R_speed = turnWeight * speed + 30 * (turnWeight * leftEnCount - rightEnCount);
  motor_R_speed = constrain(motor_R_speed, 0, 255); // Ensure PWM is in valid range

  analogWrite(enR, motor_R_speed);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void turnLeftDegrees(float degrees) {

  // Reset encoder counters
  bno.begin();
  delay(1000);

  Serial.println("ham re trai theo do");
  // Get the starting angle
  float startAngle = getCurrentYaw();
  Serial.print("goc khoi dong trong vong re trai");

  Serial.println(startAngle);
  float targetAngle =  startAngle - degrees; // Target angle is current + desired degrees
  Serial.print("goc target trong vong re trai");

  Serial.println(targetAngle);
  turnLeft(70);
  //if targetAngle > 360 => then minus 360
  if (targetAngle < 0 ) {
    targetAngle = targetAngle + 360;
    while ((getCurrentYaw() <= startAngle && getCurrentYaw() >= 0)  || (getCurrentYaw() >= targetAngle && getCurrentYaw() <= 360) ) {
      turnLeft(70);
    }

  } else {
    Serial.println("re trai theo do");

    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() > targetAngle) {
      // Print the current yaw angle
      //        Serial.print("Current Yaw Angle: ");
      //        Serial.println(getCurrentYaw());
      turnLeft(55);
    }
  }
}

bool flag_left = true;
void turnLeftTest(float angle) {
  bno.begin();
  delay(1000);
  while (flag_left) {
    float startAngle = getCurrentYaw();
    Serial.print("startAngle re phai test");
    Serial.println(startAngle);
    if (startAngle + 5 <= angle) {
      turnLeft(55);
    }
    else {
      stop();
      delay(500);
      flag_left = false;
    }

  }
}

void turnRightTest(float angle) {
  bno.begin();
  delay(1000);
  while (flag) {
    float startAngle = getCurrentYaw();
    Serial.print("startAngle re phai test");
    Serial.println(startAngle);
    if (startAngle <= angle) {
      turnRight(80);
    }
    else {
      stop();
      delay(500);
      flag = false;
    }
  }
}



float getCurrentYaw() {
  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.x; // Normalize to [0, 360)
}


void travelUntilLeftClear() {

  while (get_left_distance() < safeDistance) {
    moveForward(50); // Keep moving forward until left sensor is clear
    Serial.println("left clear");

    delay(100); // Short delay for stability

  }
  moveForward_encoder(0.3);
  stop(); // Stop once the left sensor is clear
}


void travelUntilLeftClear1() {
  moveForward_encoder(1.6);
  while (get_left_distance() < safeDistance) {
    moveForward(60); // Keep moving forward until left sensor is clear
    Serial.println("left clear");

    delay(100); // Short delay for stability
  }
  stop(); // Stop once the left sensor is clear
}

void travelUntilSeeLight() {
  if (!lineDetectedbyIRs()) {
    moveForward(60);
    Serial.println("no light");
    delay(100); // Short delay for stability
  }
  else {
    //  stop();
    Serial.println("see light trong check state");
    delay(500);
    currentState = LineFollower;
  }
}


void travelUntilRightClear() {
  while (get_right_distance() < safeDistance) {
    moveForward(60); // Keep moving forward until left sensor is clear
    moveForward_encoder(0.5);
    delay(80); // Short delay for stability

  }
  stop(); // Stop once the left sensor is clear
}


void moveForward(int speed) {
  leftEnCount = 0;
  rightEnCount = 0;

  analogWrite(enR, speed);
  int motor_L_speed = speed + 30 * (rightEnCount - leftEnCount);
  motor_L_speed = constrain(motor_L_speed, 0, 255); // Ensure PWM is in valid range
  analogWrite(enL, motor_L_speed);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

}



float get_right_distance() {
  float distance = rightSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;

  }
  return distance;


}
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
