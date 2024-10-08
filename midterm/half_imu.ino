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



const int obstacleThreshold = 15; // cm

// IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
float currentAngle = 0.0;

const int enLA = 2;
const int enLB = 3;
const int enRA = 18;
const int enRB = 19;


// Encoder counts
volatile long leftEnCount = 0;
volatile long rightEnCount = 0;
const int K = 30; 


HCSR04 frontSonar(53, 52); //initialisation class HCSR04 (trig pin , echo pin)

void setup() {
    Serial.begin(9600);
    
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

void loop() {
    // Move forward
    moveForward(60);

//    Serial.print("Distance: ");
//    Serial.println(get_front_distance());
//    
//    Serial.println(getCurrentYaw());

    
    // Check for obstacles
  if (isObstacleDetected()) {
        Serial.println("obstacle detected");
        turnRightDegrees(70);
        delay(100);
        stop();
        delay(100); // Small delay for stability
        travelUntilLeftClear();
        delay(500);
        turnLeftDegrees(70);
        delay(3000);
        
//        
        
//        travelUntilLeftClear();
//        stop();
//        
//        turnLeftDegrees(15);
//        delay(100);
//
//        travelUntilLeftClear();
//        stop();
    }
    
    
}

float get_front_distance(){
  float distance = frontSonar.dist();
  if (distance == 0.0) {
    distance = 200.0;
    
  }
  return distance; 
  
  
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

long measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2; // Convert to cm
}

bool isObstacleDetected() {
    long distance = get_front_distance();
//    Serial.print("Distance: ");
//    Serial.println(distance); // Print distance for debugging
    return distance < obstacleThreshold; // Return true if obstacle is detected
}
//
//void turnLeftDegrees(float degrees) {
//  // Reset encoder counters
//    leftEnCount = 0;
//    rightEnCount = 0;
//    digitalWrite(inL1, LOW);
//    digitalWrite(inL2, LOW);
//    digitalWrite(inR1, HIGH);
//    digitalWrite(inR2, LOW);
//
//    
//    int speed = 60;
//    const int turnWeight = 2;
//    analogWrite(enL, speed);
//
//    int motor_R_speed = turnWeight * speed + K * (turnWeight * leftEnCount - rightEnCount);
//    motor_R_speed = constrain(motor_R_speed, 0, 255); // Ensure PWM is in valid range
//    analogWrite(enR, motor_R_speed);
//    // Get the starting angle
//    currentAngle = getCurrentYaw();
//    Serial.println(currentAngle);
//    float targetAngle = currentAngle + degrees; // Target angle is current + desired degrees
//    
//    
//    
//    // Turn until the target angle is reached or timeout
//    while (getCurrentYaw() < targetAngle && millis() - startTime < 1000) {
//        // Print the current yaw angle
//        Serial.print("Current Yaw Angle: ");
//        Serial.println(getCurrentYaw());
//    }
//    
//    stop(); // Stop turning
//    delay(500); // Short delay after turning
//}

void turnRight(int speed){
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
    
  
    // Get the starting angle
    float startAngle = getCurrentYaw();
    Serial.println(startAngle);
    float targetAngle = startAngle + degrees; // Target angle is current + desired degrees
    //if targetAngle > 360 => then minus 360 
    if (targetAngle > 360){
      targetAngle = targetAngle - 360; 
      while ((getCurrentYaw() >= startAngle && getCurrentYaw() <= 360)  || (getCurrentYaw() >= 0 && getCurrentYaw() < targetAngle) ){
          turnRight(50);        
        }
      
    } else{
        
    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() < targetAngle) {
        // Print the current yaw angle
//        Serial.print("Current Yaw Angle: ");
//        Serial.println(getCurrentYaw());
       turnRight(50);
    }
    }
    stop();

}

void turnLeft(int speed){
  leftEnCount = 0;
    rightEnCount = 0;
    const int turnWeight = 2;
    analogWrite(enL, speed);

    int motor_R_speed = turnWeight * speed + K * (turnWeight * rightEnCount - leftEnCount);
    motor_R_speed = constrain(motor_R_speed, 0, 255); // Ensure PWM is in valid range
    analogWrite(enR, motor_R_speed);
        digitalWrite(inL1, LOW);
        digitalWrite(inL2, HIGH);
        digitalWrite(inR1, LOW);
        digitalWrite(inR2, HIGH);
}

void turnLeftDegrees(float degrees) {
    // Reset encoder counters
    
  
    // Get the starting angle
    float startAngle = getCurrentYaw();
    Serial.println(startAngle);
    float targetAngle = startAngle + degrees; // Target angle is current + desired degrees
    //if targetAngle > 360 => then minus 360 
    if (targetAngle > 360){
      targetAngle = targetAngle - 360; 
      while ((getCurrentYaw() >= startAngle && getCurrentYaw() <= 360)  || (getCurrentYaw() >= 0 && getCurrentYaw() < targetAngle) ){
          turnLeft(50);        
        }
      
    } else{
        
    // Turn until the target angle is reached or timeout
    while (getCurrentYaw() < targetAngle) {
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
long measureLeftDistance() {
    return measureDistance(leftTrigPin, leftEchoPin); // Measure distance from left sensor
}
long measureRightDistance() {
    return measureDistance(rightTrigPin, rightEchoPin); // Measure distance from left sensor
}


void travelUntilLeftClear() {
    while (measureLeftDistance() < obstacleThreshold) {
        moveForward(60); // Keep moving forward until left sensor is clear
        moveForward_encoder(0.8);
        
        delay(100); // Short delay for stability
    }
    stop(); // Stop once the left sensor is clear
}

void travelUntilRightClear() {
    while (measureRightDistance() < obstacleThreshold) {
        moveForward(60); // Keep moving forward until left sensor is clear
        delay(100); // Short delay for stability

    }
    stop(); // Stop once the left sensor is clear
}


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
    analogWrite(enL, 70);  // Use initial speed
    analogWrite(enR, 70);  // Use initial speed

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
