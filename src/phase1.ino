#include <Servo.h>
#include "SoftPWM.h"

// I/O pins
#define SERVO_PIN 9

#define ECHO 8
#define TRIG 7

//Currently the components are on the rover backward, hence IN2/IN1 and IN4/IN3 are switched
#define IN1 11
#define IN2 12 
#define IN3 5
#define IN4 6

#define leftIR 2
#define rightIR 3

//States
#define stopped 14
#define forward 15
#define left 16
#define right 17
#define reverse 18

const double pi = 3.1415926;

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

//Odometry
unsigned long last_debounce_timeR = 0;
unsigned long last_debounce_timeL = 0;
int irLastStateL;
int irLastStateR;
int leftCounter = 0;
int rightCounter = 0;
int lastTotalCounterL = 0;
int lastTotalCounterR = 0;
int totalCounterL = 0;
int totalCounterR = 0;
int currentCounterL = 0;
int currentCounterR = 0;
//Empirically determined constants from testing
double inches_per_tick = 0.29;
double radians_per_tick = 0.0675;
double inches_sideways_per_tick = 0.1394;
double inches_forward_per_tick = 0.2434;

// Distance from start position in inches
// X is in the forward direction from the robot's initial position/orientation
// Y is in the right direction from the robot's initial position/orientation
double posX = 0;
double posY = 0;
// Relative orientation in radians
double orientation = pi / 2.0; // Start facing "up" (pi/2, or 90 degrees)

double temp;

void goForward(){
  state = forward;
  duty_cycleA = 128;
  duty_cycleB = 128;
  SoftPWMSetPercent(IN1, 0);
  SoftPWMSetPercent(IN2, duty_cycleA);
  SoftPWMSetPercent(IN3, duty_cycleB);
  SoftPWMSetPercent(IN4, 0);
  Serial.println("Going forward");
}

void stop(){
  //state = stop;
  SoftPWMSetPercent(IN1, 0);
  SoftPWMSetPercent(IN2, 0);
  SoftPWMSetPercent(IN3, 0);
  SoftPWMSetPercent(IN4, 0);
}

void goReverse(){
  state = reverse;
  duty_cycleA = 128;
  duty_cycleB = 128;
  SoftPWMSetPercent(IN1, duty_cycleA);
  SoftPWMSetPercent(IN2, 0);
  SoftPWMSetPercent(IN3, 0);
  SoftPWMSetPercent(IN4, duty_cycleB);
  Serial.println("Reversing");
}

void turnLeft(){
  state = left;
  duty_cycleA = 165;
  duty_cycleB = 128;
  SoftPWMSetPercent(IN1, 0);
  SoftPWMSetPercent(IN2, duty_cycleA);
  SoftPWMSetPercent(IN3, 0);
  SoftPWMSetPercent(IN4, duty_cycleB);
  Serial.println("Turning left");
}

void turnRight(){
  state = right;
  duty_cycleA = 128;
  duty_cycleB = 165;
  SoftPWMSetPercent(IN1, duty_cycleA);
  SoftPWMSetPercent(IN2, 0);
  SoftPWMSetPercent(IN3, duty_cycleB);
  SoftPWMSetPercent(IN4, 0);
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
  int irStateL = digitalRead(leftIR);

  if (irStateL != irLastStateL && millis() > last_debounce_timeL + 3) {  // Only process on change
    leftCounter++;
    last_debounce_timeL = millis();
  }
  irLastStateL = irStateL;
}

void countR() {
  int irStateR = digitalRead(rightIR);

  if (irStateR != irLastStateR && millis() > last_debounce_timeR + 3) {  // Only process on change
    rightCounter++;
    last_debounce_timeR = millis();
  }
  irLastStateR = irStateR;
}

void get_position(){
  // Determine number of ticks for this iteration
  noInterrupts();
  totalCounterL = leftCounter;
  totalCounterR = rightCounter;
  interrupts();
  currentCounterL = totalCounterL - lastTotalCounterL;
  currentCounterR = totalCounterR - lastTotalCounterR;

  // Adjust for bad values. Values should be a single digit positive integer.
  if (currentCounterL > 10 || currentCounterL < 1){
    if (state == forward || state == reverse) currentCounterL = currentCounterR;
    if (state == left) currentCounterL = currentCounterR / 2;
    if (state == right) currentCounterL = currentCounterR * 2;
  }
  if (currentCounterR > 10 || currentCounterR < 1){
    if (state == forward || state == reverse) currentCounterR = currentCounterL;
    if (state == left) currentCounterR = currentCounterL / 2;
    if (state == right) currentCounterR = currentCounterL * 2;
  }

  //Calculate position based on what state we're in
  if (state == forward){
    double distance_traveled = (currentCounterL + currentCounterR) / 2.0 * inches_per_tick;
    posX = posX + cos(orientation) * distance_traveled;
    posY = posY + sin(orientation) * distance_traveled;
  }
  else if (state == reverse){
    double distance_traveled = (currentCounterL + currentCounterR) / 2.0 * inches_per_tick;
    posX = posX - cos(orientation) * distance_traveled;
    posY = posY - sin(orientation) * distance_traveled;
  }
  else if (state == left){
    double normalized_ticks = (currentCounterL + currentCounterR * 0.444) / 2.0;
    posX = posX - sin(orientation) * normalized_ticks * inches_sideways_per_tick + cos(orientation) * normalized_ticks * inches_forward_per_tick;
    posY = posY + sin(orientation) * normalized_ticks * inches_forward_per_tick - cos(orientation) * normalized_ticks * inches_sideways_per_tick;
    orientation += normalized_ticks * radians_per_tick;
  }
  else if (state == right){
    double normalized_ticks = (currentCounterL + currentCounterR * 0.444) / 2.0;
    posX = posX + sin(orientation) * normalized_ticks * inches_sideways_per_tick - cos(orientation) * normalized_ticks * inches_forward_per_tick;
    posY = posY + sin(orientation) * normalized_ticks * inches_forward_per_tick - cos(orientation) * normalized_ticks * inches_sideways_per_tick;
    orientation -= normalized_ticks * radians_per_tick;
  }

  lastTotalCounterL = totalCounterL;
  lastTotalCounterR = totalCounterR;
} 

void setup() {
  // Set pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Initialize motor state (motor off)
  SoftPWMBegin();
  SoftPWMSetPercent(IN1, 0);
  SoftPWMSetPercent(IN2, 0);
  SoftPWMSetPercent(IN3, 0);
  SoftPWMSetPercent(IN4, 0);

  // Initialize ultrasonic state
  digitalWrite(TRIG, LOW);
  
  // Initialize servo
  head.attach(SERVO_PIN);
  head.write(180); //Start in left position
  delay(200);

  // Initialize encoder interrupts
  irLastStateL = digitalRead(leftIR);
  irLastStateR = digitalRead(rightIR);
  attachInterrupt(digitalPinToInterrupt(leftIR), countL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightIR), countR, CHANGE);
  noInterrupts();
  
  Serial.begin(9600);

  backingOut = 0;
  direction = 0; // Start going from left to right
  
  stop();
  delay(200);
  turnLeft();
  interrupts();
}

void loop() {
  //Temp sensing
  /*
  temp = analogRead(A0) / 1024.0 * 5 * 100; // temperature in degrees C

  scan_range(0, 180, divisions); // This is where the most time is spent. Gathering obstacle data.

  get_position(); // Calculate our current position.

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

  Serial.print("Left counter: "); Serial.println(currentCounterL);
  Serial.print("Right counter: "); Serial.println(currentCounterR);
  Serial.print("Total left coutner: "); Serial.println(totalCounterL);
  Serial.print("Total right counter:"); Serial.println(totalCounterR);
  
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  //if (!corner) navigate(); //Hug the left wall
  */
  goForward();
  delay(1000);
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  turnLeft();
  delay(1000);
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  goForward();
  delay(1000);
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  /*
  turnRight();
  delay(1000);
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  goForward();
  delay(1000);
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  goReverse();
  delay(1000);
  */
  stop();
  get_position();
  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
  Serial.print("Orientation: "); Serial.println(orientation);
  delay(10000);
}
