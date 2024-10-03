// Motor control pins
int enL = 7; // enable the left motor driver, using PWM  
int inL1 = 8; // control direction of the left motor 
int inL2 = 9;

#define echo 53    //Echo pin
#define trigger 52 //Trigger pin
// Right Motor
int enR = 12; 
int inR1 = 10;
int inR2 = 11;

// For Encoder
int enLA = 2;  // left motor encoder
int enLB = 3;
int enRA = 18; // right motor encoder 
int enRB = 19;

//volatile int leftEncoderCount = 0;
//volatile int rightEncoderCount = 0; 
//const int targetSpeed = 40; 
const int K = 3;  // Adjust K for smooth response

// PID variables
float Kp = 4.5, Ki = 0.02, Kd = 0.5;
float sampling_rate = 0; 
float previousTime;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;

// Sensor array and motor speed
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 55;

// Function prototypes
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void setup();
void loop();

void setup() {
  
  Serial.begin(9600);
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
}
void stop(){
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void loop() {
  read_sensor_values();
 
  motor_control();
}

void read_sensor_values() {
  float currentTime = millis();
  if (currentTime - previousTime >= sampling_rate){
    for (int i = 0; i < 5; i++) {
    sensor[i] = digitalRead(A0 + i); // Assume sensors are connected to A0 to A4
  }
  //phai sang, 1 den don sang 
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)){ //00100
    error = 0;
     calculate_pid(error);}
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)){ //00010
    error = 2; 
    calculate_pid(error);}
   else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)){ //00001
    error = 4; 
    calculate_pid(error);}
   //phai sang, truong hop 2 den sang
   else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)){ //00110
    error = 2; 
    calculate_pid(error);}
   else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)){ //00011
    error = 3; 
    calculate_pid(error);}
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)){ //00101
    error = 3; 
    calculate_pid(error);}

    //trai sang, 1 den don sang
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){ //01000
    error = -2; 
    calculate_pid(error);}
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){ //10000
    error = -4; 
    calculate_pid(error);}

    //trai sang, 2 den don sang
     else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)){ //01100
    error = -1; 
    calculate_pid(error);}
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){ //11000
    error = -3; 
    calculate_pid(error);}
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)){ //10100
    error = -3; 
    calculate_pid(error);}

    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)){ //11111
    stop();}
      
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

void stop(){
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, HIGH);
}
