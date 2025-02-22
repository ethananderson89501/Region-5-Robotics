#include <Servo.h>
#include <string.h>
#include <Wire.h>

//I/O pins
#define encoderLA 3
#define encoderLB 4
#define encoderRA 12
#define encoderRB 13

#define SERVO_PIN 9

#define ECHO 8
#define TRIG 7
 
#define IN1 5
#define IN2 6 
#define IN3 3 
#define IN4 11 

//States
#define stopped 14
#define forward 15
#define left 16
#define right 17
#define reverse 18

// Servo iterations
const int divisions = 8;
// Servo direction
int direction;
Servo head;

//Motor duty cycles
int duty_cycleA;
int duty_cycleB;

//Motor states
int state;
int backingOut;

//Navigation states
int corner = 0;
int corners_checked = 0;

//Rotary encoder counter
volatile int lcounter = 0;
int laLastState;
int lcurrent_counter;
int llast_counter;
int ltotal_counter;
volatile int rcounter = 0;
int raLastState;
int rcurrent_counter;
int rlast_counter;
int rtotal_counter;

//Odometry variables
const double dia = 20.75; // Wheel diameter in inches  
double Dl, Dr, Dc, Ori_ch;
const int ER = 24; // Pulses per revolution
double Ori  = 0; // Orientation in radians
const double dist_const = dia * 3.1415926;
double b = 20; // Wheelbase. Need to research/test this.

// Distance from start position in inches
double posX = 0;
double posY = 0;

double temp;

void goForward(){
  state = forward;
  duty_cycleA = 128;
  duty_cycleB = 128;
  analogWrite(IN1, 0);
  analogWrite(IN2, duty_cycleA);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
  Serial.println("Going forward");
}

void stop(){
  state = stop;
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void goReverse(){
  state = reverse;
  duty_cycleA = 128;
  duty_cycleB = 128;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, duty_cycleB);
  Serial.println("Reversing");
}

void turnLeft(){
  state = left;
  duty_cycleA = 192;
  duty_cycleB = 128;
  analogWrite(IN1, 0);
  analogWrite(IN2, duty_cycleA);
  analogWrite(IN3, 0);
  analogWrite(IN4, duty_cycleB);
  Serial.println("Turning left");
}

void turnRight(){
  state = right;
  duty_cycleA = 128;
  duty_cycleB = 192;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
  Serial.println("Turning right");
}

double watch(){
  digitalWrite(TRIG,LOW);
  delayMicroseconds(5);                                                                            
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10); // give or take 7 feet
  digitalWrite(TRIG,LOW);
  long echo_time=pulseIn(ECHO,HIGH);
  if (echo_time <= 0) {
    return -1.0; // Return a special value to indicate no echo detected
  }
  double echo_distance=echo_time / 148.0; //how far away is the object in inches
  return echo_distance;
}

double dists[divisions + 1];
void scan_range(double theta_0, double theta_1, int divisions){
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions; i++){
    double angle = theta_0 + i * delta; // 2 * theta_1 since divisions is 2 * on second pass
    if (direction == 1) head.write(angle);
    else head.write(theta_1 - angle);
    delay(30);
    double dist = watch();
    if (dist < 0) dist = 0; // Filters out invalid values
    if (direction == 1) dists[i] = dist;
    else dists[divisions - i] = dist;
  }
  if (direction == 0) direction = 1;
  else direction = 0;
}

double getMin(double* array, int size) {
  double min = array[0];
  for (int i = 1; i < size; i++) {
    if (array[i] < min) min = array[i];
  }
  return min;
}

double center_array[divisions / 3];
double left_array[divisions / 3];
double right_array[divisions / 3];
double center_min;
double right_min;
double left_min;

void navigate(){
  // Find minimum value in the center
  for (int i = 0; i <= divisions / 3; i++){
    center_array[i] = dists[i + divisions / 3 + 1];
  }
  center_min = getMin(center_array, divisions / 3 + 1);

  // Find minimum value on the left side
  for (int i = 0; i <= divisions / 3; i++){
    left_array[i] = dists[i + divisions *2/3 + 1];
  }
  left_min = getMin(left_array, divisions / 3 + 1);

  // Find minimum value on the right side
  for (int i = 0; i <= divisions / 3; i++){
    right_array[i] = dists[i];
  }
  right_min = getMin(right_array, divisions / 3 + 1);

  // Logic to hug the left wall
  if (state == forward){
      if (center_min >= 7 && left_min < 10 && left_min > 4 && right_min > 4){
        goForward();
      }else if (left_min >= 10 && right_min > 4){
        turnLeft();
      }
      /*
      else if (backingOut && center_min >= 7) goForward();
      else if (backingOut && left_min < 10){
        backingOut = 0;
        goForward();
      }
      */
      else if (right_min >= 7 & left_min > 4){
        turnRight();
      }
      else{
        backingOut = 1;
        goReverse();
        Serial.println("Backing out");
      }
  }
  else if (state == right){
    if (center_min < 7){
      turnRight();
    }
    else if (backingOut && right_min >= 12 && center_min < 10) turnRight();
    else {
      goForward();
      backingOut = 0;
    }
  }
  else if (state == left){
    if (left_min >= 10){
      turnLeft();
    }
    else{ goForward();}
  }
  
  else if (state == reverse){
    if (right_min >= 8){
      turnRight();
      backingOut = 1;
    }
    else goReverse();
  }
}

void countL() {
  int laState = digitalRead(encoderLA);
  int lbState = digitalRead(encoderLB);

  if (laState != laLastState) {  // Only process on change
    if (lbState != laState) lcounter++;
    else lcounter--;
  }
  laLastState = laState;
}
void countR() {
  int raState = digitalRead(encoderRA);
  int rbState = digitalRead(encoderRB);

  if (raState != raLastState) {  // Only process on change
    if (rbState != raState) rcounter++;
    else rcounter--;
  }
  raLastState = raState;
}

void get_position(){
  // Determine number of ticks for this iteration
  noInterrupts();
  ltotal_counter = lcounter;
  rtotal_counter = rcounter;
  interrupts();
  lcurrent_counter = ltotal_counter - llast_counter;
  llast_counter = ltotal_counter;
  rcurrent_counter = rtotal_counter - rlast_counter;
  rlast_counter = rtotal_counter;
  
  Dl = dist_const * lcurrent_counter; // Change in left distance
  Dr = dist_const * rcurrent_counter; // Change in right distance 
  Dc = (Dl + Dr) / 2 ; // Displacement
  Ori_ch = (Dr - Dl) / b; // Change in orientation
  Ori = Ori + Ori_ch; // New orientation
  posX = posX + Dc * cos(Ori); 
  posY = posY + Dc * sin(Ori);
}

void setup() {
  // Set pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(encoderLA, INPUT_PULLUP);
  pinMode(encoderLB, INPUT_PULLUP);
  pinMode(encoderRA, INPUT_PULLUP);
  pinMode(encoderRB, INPUT_PULLUP);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Initialize motor state (motor off)
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  // Initialize ultrasonic state
  digitalWrite(TRIG, LOW);
  
  // Initialize servo
  head.attach(SERVO_PIN);
  head.write(180); //Start in left position
  delay(200);

  // Initialize encoder interrupts
  laLastState = digitalRead(encoderLA);
  llast_counter = 0;
  attachInterrupt(digitalPinToInterrupt(encoderLA), countL, CHANGE);
  raLastState = digitalRead(encoderRA);
  rlast_counter = 0;
  attachInterrupt(digitalPinToInterrupt(encoderRA), countR, CHANGE);
  
  Serial.begin(9600);

  backingOut = 0;
  direction = 0; // Start going from left to right
  
  stop();
  delay(200);
  goForward();
}

void loop() {
  //Temp sensing
  temp = analogRead(A0) / 1024.0 * 5 * 100; // temperature in degrees C
  
  // Calculate our position
  get_position();

  // Check to see if we've found the next corner
  if (!corner){
    if (corners_checked == 0){ // Target: 1ft, 7ft
      if (posY > 6*12) corner = 1; //If we've gone forward 6 ft, find the corner
    }
    if (corners_checked == 1){ //Target: 7ft, 7ft
      if (posX > 6*12) corner = 1; //If we've traveled to the first corner, and then right 6 ft, find the corner
    }
    if (corners_checked == 2){ //Target: 7ft, 1ft
      if (posY < 1*12) corner = 1; //If we're within 1 ft of our original y position, find the corner
    }
  }

  scan_range(0, 180, divisions); // This is where the most time is spent
  if (!corner) navigate(); //Hug the left wall
}
