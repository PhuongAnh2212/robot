#include <Wire.h>

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
const int turnSpeed = 100; // PWM value for motors
const int obstacleThreshold = 30; // cm

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
}

void loop() {
    moveForward();
    
    long distance = measureDistance(frontTrigPin, frontEchoPin);
    Serial.print("Distance: ");
    Serial.println(distance);
    
    if (distance < obstacleThreshold) {
        stop();
        delay(500); // Stop for half a second
        // You could add turning logic here if you want
    }
    
    delay(100); // Small delay to prevent overwhelming the Serial output
}

void stop() {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);
}

void moveForward() {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
    analogWrite(enL, turnSpeed); 
    analogWrite(enR, turnSpeed);
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
