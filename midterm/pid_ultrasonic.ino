#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <HCSR04.h>

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

const int obstacleThreshold = 25; // cm
const int safeDistance = 15;

const int enLA = 2;
const int enLB = 3;
const int enRA = 18;
const int enRB = 19;

float Kp_sona = 6.0, Ki_sona = 1.0, Kd_sona = 4.0;

// Variables for PID controller
float P_sona = 0, I_sona = 0, D_sona = 0;
float previous_error_sona = 0;
float PID_value_sona = 0;
float setpoint = 20.0;

float previousTime_sona = 0.0;
float sampling_rate_sona = 1.0; //don vi la miliseconds

int initial_motor_speed = 45;


// Encoder counts
volatile long leftEnCount = 0;
volatile long rightEnCount = 0;
const int K = 30;

// IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
float currentAngle = 0.0;

HCSR04 frontSonar(53, 52); //initialisation class HCSR04 (trig pin , echo pin)
HCSR04 leftSonar(49, 48); //initialisation class HCSR04 (trig pin , echo pin)
HCSR04 rightSonar(51, 50); //initialisation class HCSR04 (trig pin , echo pin)

enum RobotState {
  Normal,
  AvoidObstacle
};

RobotState currentState = Normal;

bool a;
bool x = false;
bool b = false;


void setup() {
  Serial.begin(9600);
  Serial.println("Setup is complete."); // This will print to the Serial Monitor

  // Initialize IMU
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Set motor pins as output
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

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
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISR, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISR, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISR, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISR, RISING);

  stop();
}

// void check_sona() {
//   // Check the state of obstacles
//   bool isFrontObstacle = ObstacleFront();

//   if (isFrontObstacle()) {
//     Serial.println("obstacle detected");
//     turnRightDegrees(65);

//     delay(1000);
//     pid_obstacle();


//     delay(300);
//     travelUntilLeftClear1();
//     delay(300);
//     turnLeftDegrees(65);
//     }
// }

void loop() {
  //  switch (currentState) {
  //      case Normal:
  //          checkNormalState();
  //          break;
  //          Serial.println("Normal State");
  //
  //      case AvoidObstacle:
  //          checkAvoidObstacleState();
  //          break;
  //          Serial.println("Obstacle State");
  //
  //    }
  moveForward(60);
  checkNormalState();

  if (a == true) {
    analogWrite(enR, 0);
    analogWrite(enL, 65);
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);


    delay(1000);
    b = true;
    a = false;
  }

  if (b == true) {
    read_sona_values();
  }
}

void checkNormalState() {
  Serial.println("Loop is running."); // This will print repeatedly

  // Move forward at a specified speed
  //    moveForward(55);

  // Check for obstacles in front
  if (ObstacleFront()) {
    Serial.println("Obstacle detected, switching to AvoidObstacle state.");
    currentState = AvoidObstacle;  // Transition to avoid obstacle state
    a = true;
  }
}

void checkAvoidObstacleState() {
  // Turn right to avoid the obstacle
  turnRightDegrees(45);
  delay(3000);  // Allow time for the turn

  // Call PID controller to adjust left and right motors based on sonar readings
  read_sona_values();  // This function will handle PID adjustments
  motor_control_sona(55);

  delay(1000);
  // Check again for obstacles after adjustment
  if (!ObstacleFront() && !ObstacleLeft()) {
    currentState = Normal;  // Return to normal state if no obstacles are detected
  }
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

float get_right_distance() {
  float distance = rightSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;

  }
  return distance;

}

bool ObstacleFront() {
  long distanceFront = get_front_distance();
  return distanceFront < obstacleThreshold; // Return true if obstacle is detected
}

bool ObstacleLeft() {
  long distanceLeft = get_left_distance();
  return distanceLeft < obstacleThreshold; // Return true if obstacle is detected
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  delay(500);
}

void moveForward(int speed) {
  leftEnCount = 0;
  rightEnCount = 0;

  analogWrite(enR, speed);
  int motor_L_speed = speed + K * (rightEnCount - leftEnCount);
  motor_L_speed = constrain(motor_L_speed, 0, 255); // Ensure PWM is in valid range
  analogWrite(enL, motor_L_speed);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

}

void turnRight(int speed) {
  leftEnCount = 0;
  rightEnCount = 0;
  const int turnWeight = 1;
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
  // Get the starting angle
  float startAngle = getCurrentYaw();
  Serial.println(startAngle);
  float targetAngle = startAngle + degrees; // Target angle is current + desired degrees
  stop();
  bno.begin();
  delay(1000);
  //if targetAngle > 360 => then minus 360
  //    if (targetAngle > 360 && x == false){
  //      targetAngle = targetAngle - 360;
  //      while ((getCurrentYaw() >= startAngle && getCurrentYaw() <= 360)  || (getCurrentYaw() >= 0 && getCurrentYaw() < targetAngle) ){
  //          turnRight(150);
  //      }
  //      x = true;
  //    }
  if (targetAngle <= 360) {
    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() < targetAngle) {
      // Print the current yaw angle
      Serial.print("Current Yaw Angle: ");
      //        Serial.println(getCurrentYaw());
      turnRight(100);
    }
  }
  stop();

}

void turnLeft(int speed) {
  //  leftEnCount = 0;
  //    rightEnCount = 0;
  //    const int turnWeight = 2;
  //    analogWrite(enL, speed);
  //
  ////    int motor_R_speed = turnWeight * speed + K * (turnWeight * leftEnCount - rightEnCount);
  ////    motor_R_speed = constrain(motor_R_speed, 0, 255); // Ensure PWM is in valid range
  //    int motor_R_speed = 100;
  analogWrite(enR, 100);
  analogWrite(enL, 100);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void turnLeftDegrees(float degrees) {

  // Get the starting angle
  float startAngle = getCurrentYaw();
  Serial.println(startAngle);
  float targetAngle =  startAngle - degrees; // Target angle is current + desired degrees
  Serial.println(targetAngle);
  turnLeft(50);
  //if targetAngle > 360 => then minus 360
  if (targetAngle < 0 ) {
    targetAngle = targetAngle + 360;
    while ((getCurrentYaw() <= startAngle && getCurrentYaw() >= 0)  || (getCurrentYaw() >= targetAngle && getCurrentYaw() <= 360) ) {
      turnLeft(50);
    }

  } else {
    Serial.println("hehe");

    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() > targetAngle) {
      // Print the current yaw angle
      //        Serial.print("Current Yaw Angle: ");
      //        Serial.println(getCurrentYaw());
      turnLeft(50);
    }
  }
  stop();

}

float getCurrentYaw() {
  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.x; // Normalize to [0, 360)
}


// void travelUntilLeftClear() {
//         moveForward_encoder(0.3);
//     while (get_left_distance() < 35) {
//         moveForward(70); // Keep moving forward until left sensor is clear
//         Serial.println("left clear");

//         delay(100); // Short delay for stability
//     }
//     stop(); // Stop once the left sensor is clear
// }
// void travelUntilLeftClear1() {
//         moveForward_encoder(1.0);
//     while (get_left_distance() < 35) {
//         moveForward(70); // Keep moving forward until left sensor is clear
//         Serial.println("left clear");

//         delay(100); // Short delay for stability
//     }
//     stop(); // Stop once the left sensor is clear
// }
// void travelUntilRightClear() {
//     while (get_right_distance() < obstacleThreshold) {
//         moveForward(60); // Keep moving forward until left sensor is clear
//         moveForward_encoder(0.3);
//         delay(80); // Short delay for stability

//     }
//     stop(); // Stop once the left sensor is clear
// }


// Encoder ISR for left motor
void leftEnISR() {
  leftEnCount++;
}

void rightEnISR() {
  rightEnCount++;
}

void moveForward_encoder(float distance_en) {
  int targetCounts = calculateTargetCounts(distance_en);
  leftEnCount = 0; // Reset encoder counts
  rightEnCount = 0;

  // Set direction for moving forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);

  // Start moving forward
  analogWrite(enL, 55);  // Use initial speed
  analogWrite(enR, 55);  // Use initial speed

  // Wait until the robot has moved the target distance
  while (leftEnCount < targetCounts && rightEnCount < targetCounts) {
    // Optionally add obstacle checking code here
  }

  // Stop motors after moving forward
  stop();
}
int calculateTargetCounts(float distance_en) {
  const int encoderCountsPerRevolution = 500; // 1:50 encoder with 7 pulses per revolution
  const float wheelDiameter = 0.0225 * 2; // Diameter of the wheel in meters
  const float wheelCircumference = wheelDiameter * M_PI; // Wheel circumference
  return (distance_en / wheelCircumference) * encoderCountsPerRevolution;
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

void motor_control_sona(float PID_value_sona) {
  // Calculating the effective motor speed:
  int left_motor_speed = abs(initial_motor_speed - (PID_value_sona/100));
  int right_motor_speed = abs(initial_motor_speed + (PID_value_sona/100));

  left_motor_speed = constrain(left_motor_speed, 20, 100);
  right_motor_speed = constrain(right_motor_speed, 20, 100);

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

void read_sona_values() {
  //  float currentTime_sona = millis();

  // Check if it's time to read new sonar values based on sampling rate
  //  if (currentTime_sona - previousTime_sona >= sampling_rate_sona) {
  //      previousTime_sona = currentTime_sona;

  // Get distance values from left and right sonar sensors
  long distanceLeft = get_left_distance();
  long distanceRight = get_right_distance();

  Serial.println("Distance Left:");
  Serial.println(distanceLeft);

  // Check if there are obstacles and calculate PID if needed
  if (distanceLeft < safeDistance) {
    float error_sona = safeDistance - distanceLeft; // Error = difference in distances
    float pid = calculate_pid_sona(error_sona); // Call the PID controller with the calculated error
    motor_control_sona(PID_value_sona);
    Serial.println("meowssssss");

  }
  else if (distanceLeft >= safeDistance) {
    float error_sona = distanceLeft - safeDistance; // Error = difference in distances
    float pid = calculate_pid_sona(error_sona); // Call the PID controller with the calculated error
    motor_control_sona(pid);
    Serial.println("ssssssswoem");

  }
  //    }
}
