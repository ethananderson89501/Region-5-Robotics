#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <NewPing.h>

// I/O pins
#define SERVO_PIN 9

#define ECHO 8
#define TRIG 7

#define IN1 3
#define IN2 11 
#define IN3 5
#define IN4 6

#define leftIR 2
#define rightIR 4

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

// Ultrasonic
int MAX_DISTANCE = 250;
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

//Navigation states
int corner = 0;
int corners_checked = 0;
int checking_state = 0;
int target_found = 0;
int checking_counter = 0;

//Odometry
volatile unsigned long last_debounce_timeR = 0;
volatile unsigned long last_debounce_timeL = 0;
int irLastStateL;
int irLastStateR;
volatile int leftCounter = 0;
volatile int rightCounter = 0;
int lastTotalCounterL = 0;
int lastTotalCounterR = 0;
int totalCounterL = 0;
int totalCounterR = 0;
int currentCounterL = 0;
int currentCounterR = 0;
//Empirically determined constants from testing
//double inches_per_tick = 0.29; // Old for clean treads
double inches_per_tick = .346; // Measured from dirty treads
double radians_per_tick = 0.0216; // Measured from dirty treads
double turning_radius = 3;

// Distance from start position in inches
// X is in the forward direction from the robot's initial position/orientation
// Y is in the right direction from the robot's initial position/orientation
double posX = 0;
double posY = 0;
// Relative orientation in radians
double orientation = pi / 2.0; // Start facing "up" (pi/2, or 90 degrees)

double temp;

unsigned long current_time;
unsigned long elapsed_time1 = 0;
unsigned long elapsed_time2 = 0;
unsigned long elapsed_time_temp = 0;
unsigned long last_time1;
unsigned long last_time2;
unsigned long last_time_temp;
int no_print;

int nav_state = 0;

bool checkShouldExitObstacleAboidance(double angleto, double orientation){
  head.write(angleto - orientation);
  delay(30);
  if watch() < 5{
    return true;
  }else{
    return false;
  }
}

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
  //state = stop;
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
  duty_cycleA = 128;
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
  duty_cycleB = 191;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
  Serial.println("Turning right");
}

double watch(){
  double echo_distance = sonar.ping_in();
  Serial.print("Distance: "); Serial.println(echo_distance);
  return echo_distance;
}

double dists[divisions + 1];
void scan_range(double theta_0, double theta_1, int divisions){
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions; i++){
    double angle = theta_0 + i * delta; // 2 * theta_1 since divisions is 2 * on second pass
    if (direction == 1) head.write(angle);
    else head.write(theta_1 - angle);
    last_time1 = micros();
    while (micros() < last_time1 + 30000){} // delay 
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

void getMins(){
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
  Serial.print("Distances: ");
  for (int i = 0; i < 9; i++){
    Serial.print(dists[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Mins: "); Serial.print(left_min); Serial.print(" "); Serial.print(center_min); Serial.print(" "); Serial.println(right_min);
}

void navigate(){  // Logic to hug the left wall
    if (state == forward){
      if (center_min >= 10 && left_min < 30 && left_min > 4 && right_min > 4){
        goForward();
      }else if (left_min >= 10 && right_min > 4){
        turnLeft();
      }
      
      else if (backingOut && center_min >= 7) goForward();
      else if (backingOut && left_min < 10){
        backingOut = 0;
        goForward();
      }
      
      else if (right_min >= 8 & left_min > 4){
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
    if (left_min >= 30){
      turnLeft();
    }
    else if (center_min > 7) goForward();
    else if (right_min > 7) turnRight();
    else{
      backingOut = 1;
      goReverse();
    }
  }
  
  else if (state == reverse){
    if (right_min >= 8){
      turnRight();
      backingOut = 1;
    }
    else goReverse();
  }
}

double targetX;
double targetY;
double target_angle;

void findCorner(){
  Serial.print("State: "); Serial.println(checking_state);

  // State 0: Find the target
  if (checking_state == 0){
    // Target = (5, 5) away from the heat gun
    if (corners_checked == 0){ // No corners checked: goal is directly forward from start point
      targetX = 17;
      targetY = 8*12 - 17; 
    }
    else if (corners_checked == 1){ // One corner checked: goal is opposite from start point
      targetX = 8*12 - 17;
      targetY = 8*12 - 17;
    }
    else if (corners_checked == 2){ // Two corners checked: goal is directly right from start point
      targetX = 8*12 - 17;
      targetY = 17;
    }
    else{
      Serial.println("Error: all corners have been checked");
    }
    target_angle = atan((targetY - posY) / (targetX - posX));

    checking_state = 1;
  }

  // State 1: Get proper orientation
  if (checking_state == 1){
    Serial.print("Target Angle: "); Serial.println(target_angle);
    Serial.print("Orientation: "); Serial.println(orientation);
    if (target_angle - orientation < 10.0 / 180.0 * pi && target_angle - orientation > -10.0 / 180.0 * pi){ // If we're within 10 degrees of the target angle, start going forward
      checking_state = 2;
    }
    // Otherwise, turn left or right depending on which is closer
    else if (target_angle - orientation < 0) turnRight();
    else if (target_angle - orientation > 0) turnLeft();
  }

  // State 2: Traveling toward the goal
  if (checking_state == 2){
    if (posX > targetX - 2 && posX < targetX + 2 && posY > targetY - 2 && posY < targetY + 2){ // If we're at the target within +/- 2 inches
      checking_state = 4;
      last_time_temp = micros();
    }
    else if (center_min < 5){
      goForward();
    }
    else checking_state = 4;
  }

  // State 3: Obstacle avoidance
  if (checking_state == 3){
    if(checking_counter >= 10){ // Try to avoid obstacles for five iterations through the loop, then try again
      checking_counter = 0;
      checking_state = 1;
    }
    else{
      checking_counter++;
      navigate();
    }
  }

  // State 4: Checking temperature
  if (checking_state == 4){
    stop();
    if (corners_checked == 2) target_found = 1;
    else{
      temp = analogRead(A0) / 1024.0 * 5 * 100; // temperature in degrees C
      if (temp > 35){ // Adjust temperature threshold as neccessary
        target_found = 1;
      }
      else{
        current_time = micros();
        elapsed_time_temp = current_time - last_time_temp;
        if (elapsed_time_temp > 3000000){ // Stay for three seconds to allow temperature sensor to adjust
          corners_checked++;
          corner = 0;
        }
      }
    }
  }

}

void countL() {
  current_time = micros();
  if (current_time > last_debounce_timeL + 3000) {  // Only process on change
    leftCounter++;
    last_debounce_timeL = current_time;
  }  
}

void countR() {
  current_time = micros();
  if (current_time > last_debounce_timeR + 3000) {  // Only process on change
    rightCounter++;
    last_debounce_timeR = current_time;
  }  
}

void get_position(){
  // Determine number of ticks for this iteration
  noInterrupts();
  totalCounterL = leftCounter;
  totalCounterR = rightCounter;
  interrupts();
  currentCounterL = totalCounterL - lastTotalCounterL;
  currentCounterR = totalCounterR - lastTotalCounterR;

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
    double dori = (currentCounterL + currentCounterR) * radians_per_tick;
    posX = posX + turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    posY = posY + turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);;
    orientation += dori;
  }
  else if (state == right){
    double dori = -1 * (currentCounterL + currentCounterR) * radians_per_tick;
    posX = posX + turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    posY = posY + turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);;
    orientation += dori;
  }
  
  if (orientation > pi) orientation -= 2*pi;

  lastTotalCounterL = totalCounterL;
  lastTotalCounterR = totalCounterR;

  Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
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
  irLastStateL = digitalRead(leftIR);
  irLastStateR = digitalRead(rightIR);
  attachInterrupt(digitalPinToInterrupt(leftIR), countL, CHANGE);
  attachPCINT(digitalPinToPCINT(rightIR), countR, CHANGE);
  
  Serial.begin(9600);

  backingOut = 0;
  direction = 0; // Start going from left to right
  
  no_print = 0;

  stop();
  delay(200);
  last_time1 = micros();
  last_time2 = micros();
  goForward();
}

void loop() {
  
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

  getMins();
  if (!corner) navigate(); //Hug the left wall
  else findCorner();

  ////////////// Navigation testing logic //////////////////////////
  /*
  get_position();
  current_time = micros();
  elapsed_time1 = current_time - last_time1;
  elapsed_time2 = current_time - last_time2;

  if (elapsed_time1 > 500000 && !no_print){
    get_position();
    //Serial.print("Total left coutner: "); Serial.println(totalCounterL);
    //Serial.print("Total right counter:"); Serial.println(totalCounterR);
    Serial.print("Left counter: "); Serial.println(currentCounterL);
    Serial.print("Right counter: "); Serial.println(currentCounterR);
    Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
    Serial.print("Orientation: "); Serial.println(orientation * 180 / pi);
    last_time1 = current_time;
    //no_print = 1;
  }

  if (current_time > 3000000 && !no_print){
    stop();
    get_position();
    Serial.print("Total left coutner: "); Serial.println(totalCounterL);
    Serial.print("Total right counter:"); Serial.println(totalCounterR);
    Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
    Serial.print("Orientation: "); Serial.println(orientation * 180 / pi);
    no_print = 1;
  }
  

  if (elapsed_time2 > 10000){
    if (nav_state == 0){
      goForward();
      if (posY > 6*12) nav_state = 3;
    }
    if (nav_state == 1){
      turnRight();
      if (orientation < 0) nav_state = 2;
    }
    if (nav_state == 2){
      goForward();
      if (posX > 6*12){
        nav_state = 3;
      }
    }
    if (nav_state == 3){
      findCorner();
    }
  }
  */
}
