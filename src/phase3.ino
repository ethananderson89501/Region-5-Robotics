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

int target_corner = 0; // Select this value based on round 1

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
int checking_state = 0;
int target_found = 0;
int checking_counter = 0;
int finding_state = 0;
int stuck_counter = 0;
int finished = 0;
int latched = 0;
int latching_state = 0;

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
//double radians_per_tick = 0.0199; //Clean treads
//double inches_per_tick = .346; // Measured from dirty treads
//double radians_per_tick = 0.0216; // Measured from dirty treads
//double turning_radius = 3;
//double inches_per_tick = 0.307; //Tested 3/15
//double radians_per_tick = 0.0209; // Tested 3/15
//double inches_per_tick = 0.0700; // 3/16 both sensors - right reading more
//double radians_per_tick = 0.00786; // 3/16 both sensors - right reading more
//double inches_per_tick = 0.129; // 3/16 right sensor only
//double radians_per_tick = .0116; // 3/16 right sensor only
double inches_per_tick = 0.139; // 3/16 both sensors reading the same
double radians_per_tick = 0.0188; // 3/16 both sensors reading the same

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
int no_print;

int nav_state = 0;

bool checkShouldExitObstacleAboidance(double angleto, double orientation){
  head.write(angleto - orientation);
  delay(30);
  if (watch() < 5){
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
  duty_cycleB = 128;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
  Serial.println("Turning right");
}

double watch(){
  double echo_distance = sonar.ping_in();
  if (echo_distance == 0){
    echo_distance = 100;
    Serial.println("Failed reading");
  }
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
    while (micros() < last_time1 + 40000){} // delay 
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

double center_array[divisions / 3 + 1];
double left_array[divisions / 3 + 1];
double right_array[divisions / 3 + 1];
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
  if (state == stop){
    findWall();
    finding_state = 1;
  }
  if (state == forward){
    if (center_min >= 9 && left_min < 12 && left_min > 6 && right_min > 6){
      goForward();
    }else if (left_min >= 12 && right_min > 6){
      turnLeft();
    }
    /*
    else if (backingOut && center_min >= 7) goForward();
    else if (backingOut && left_min < 10){
      backingOut = 0;
      goForward();
    }
    */
    else if (right_min >= 9 & left_min > 6){
      turnRight();
    }
    else{
      backingOut = 1;
      goReverse();
      Serial.println("Backing out");
    }
  }
  else if (state == right){
    if (center_min < 9){
      turnRight();
    }
    else if (backingOut && right_min >= 14 && center_min < 12) turnRight();
    else {
      goForward();
      backingOut = 0;
    }
  }
  else if (state == left){
    if (left_min >= 12){
      turnLeft();
    }
    //else if (left_min > 4) { goForward();}
    //else if (right_min >= 7) turnRight();
    else goForward();// goReverse();
  }
  
  else if (state == reverse){
    if (right_min >= 10){
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
  // State 0: Find the target
  if (checking_state == 0){
    // Target = (5, 5) away from the heat gun
    if (target_corner == 1){ // No corners checked: goal is directly forward from start point
      targetX = 8;
      targetY = 6*12; 
    }
    else if (target_corner == 2){ // One corner checked: goal is opposite from start point
      targetX = 8 + 5*12;
      targetY = 6*12;
    }
    else if (target_corner == 3){ // Two corners checked: goal is directly right from start point
      targetX = 8 + 5*12;
      targetY = 1*12;
    }
    else{
      Serial.println("Error: all corners have been checked");
    }
    checking_state = 1;
  }

  Serial.print("State: "); Serial.println(checking_state);
  target_angle = atan((targetY - posY) / (targetX - posX));
  if (targetX - posX < 0) target_angle = target_angle * -1; // Flip sign if we're in quadrants 2 or 3
  Serial.print("Target Angle: "); Serial.println(target_angle);

  // State 1: Get proper orientation
  if (checking_state == 1){
    Serial.print("Orientation: "); Serial.println(orientation);
    if (target_angle - orientation < 10.0 / 180.0 * pi && target_angle - orientation > -10.0 / 180.0 * pi){ // If we're within 10 degrees of the target angle, start going forward
      checking_state = 2;
      stuck_counter = 0;
    }
    // Otherwise, turn left or right depending on which is closer
    else if (target_angle - orientation < 0) turnRight();
    else if (target_angle - orientation > 0) turnLeft();
  }

  // State 2: Traveling toward the goal
  if (checking_state == 2){
    if (posX > targetX - 3 && posX < targetX + 3 && posY > targetY - 3 && posY < targetY + 3){ // If we're at the target within +/- 2 inches
      stop();
      target_found = 1;
      stuck_counter = 0;
    }
    else if (abs(target_angle - orientation) > pi/2) checking_state = 1;
    else if (center_min > 7){
      goForward();
    }
    else{
      checking_state = 3; // Obstalce avoidance
      stuck_counter = 0;
    }
    //else checking_state = 4; // Just stop if you see an obstacle
  }

  // State 3: Obstacle avoidance
  
  if (checking_state == 3){
    if (posX > targetX - 5 && posX < targetX + 5 && posY > targetY - 5 && posY < targetY + 5){ // If we're at the target within +/- 2 inches
      stop();
      target_found = 1;
      stuck_counter = 0;
    }
    else if(checking_counter >= 5 || posX > targetX + 10 || posX < targetX - 10 || posY > targetY + 10 || posY < targetY - 10){ // Try to avoid obstacles for five iterations through the loop, then try again
      checking_counter = 0;
      checking_state = 1;
      stuck_counter = 0;
    }
    else{
      checking_counter++;
      navigate();
    }
  }

  stuck_counter++;
  if (stuck_counter > 10){ // Adjust this value while testing
    stuck_counter = 0;
    checking_state = 4;
  }
}

void findWall(){
  Serial.println("Finding wall");

  if (target_corner == 1) target_angle = pi/2;
  if (target_corner == 2) target_angle = 0;
  if (target_corner == 3) target_angle = -pi/2;
  Serial.print("Target angle: "); Serial.println(target_angle);

  // State 1: target angle
  if (finding_state == 1){
    Serial.print("Orientation: "); Serial.println(orientation);
    if (target_angle - orientation < 10.0 / 180.0 * pi && target_angle - orientation > -10.0 / 180.0 * pi){ // If we're within 10 degrees of the target angle, start going forward
      finding_state = 2;
    }
    // Otherwise, turn left or right depending on which is closer
    else if (target_angle - orientation < 0) turnRight();
    else if (target_angle - orientation > 0) turnLeft();
  }

  // State 2: Moving forward
  if (finding_state == 2){
    goForward();
    if (center_min < 12 || left_min < 12 || right_min < 12){
      finding_state = 0;
      Serial.println("Returning to normal");
    }
  }

}

void countL() {
  current_time = micros();
  if (current_time > last_debounce_timeL + 10000) {  // Only process on change
    leftCounter++;
    last_debounce_timeL = current_time;
  }  
}

void countR() {
  current_time = micros();
  if (current_time > last_debounce_timeR + 10000) {  // Only process on change
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
    double distance_traveled = (currentCounterL + currentCounterR) * inches_per_tick;
    posX = posX + cos(orientation) * distance_traveled;
    posY = posY + sin(orientation) * distance_traveled;
  }
  else if (state == reverse){
    double distance_traveled = (currentCounterL + currentCounterR) * inches_per_tick;
    posX = posX - cos(orientation) * distance_traveled;
    posY = posY - sin(orientation) * distance_traveled;
  }
  else if (state == left){
    double dori = (currentCounterL + currentCounterR) * radians_per_tick;
    //posX = posX + turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    //posY = posY + turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);;
    orientation += dori;
  }
  else if (state == right){
    double dori = -1 * (currentCounterL + currentCounterR) * radians_per_tick;
    //posX = posX + turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    //posY = posY + turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);;
    orientation += dori;
  }
  
  if (orientation > pi) orientation -= 2*pi;
  if (orientation < -1*pi) orientation += 2*pi;

  lastTotalCounterL = totalCounterL;
  lastTotalCounterR = totalCounterR;

  //Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
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
  if (!latched){
    get_position();

    if (latching_state == 0){ // Travleing toward the latch
      scan_range(0, 180, divisions);
      if (posY > 3*12){
        latching_state = 2;
      }
      else if (center_min < 7){
        latching_state = 1;
      }
      else goForward();
    }
    if (latching_state == 1){ // Navigating around an obstacle
      scan_range(0, 180, divisions);
      navigate();
      if (checking_counter == 0) checking_counter = 1;
      else if (orientation > 85.0 / 180.0 * pi && orientation < 95.0 / 180.0 * pi){
        latching_state = 0;
        checking_counter = 0;
        goForward();
      }
    }
    if (latching_state == 2){ // Getting correct x position
      if (posX < 8){
        if (orientation > 2.0 / 180.0 * pi) turnRight();
        else goForward();
      }
      else if (posX > 12){
        if (abs(orientation) < 178.0 / 180.0 * pi) turnLeft();
        else goForward();
      }
      else latching_state = 3;
    }
    if (latching_state == 3){// Get in reverse position
      if (-1*pi - orientation < -2.0 / 180.0 * pi) turnRight();
      else if (-1*pi - orientation > 2.0 / 180.0 * pi) turnLeft();
      else latching_state = 4;
    }
    if (latching_state == 4){ // Reverse
      if (posY > 4*12 + 3){
        posY -= 3; // Account for slipping from the ground while driving the carabiner into the latch
        latched = 1;
      }
      else{
        goReverse();
      }
    }
  }

  else if (!target_found){
    scan_range(0, 180, divisions); // This is where the most time is spent. Gathering obstacle data.

    get_position(); // Calculate our current position.

    // Check to see if we've found the next corner
    if (!corner){
      if (target_corner == 1){ // Target: 1ft, 7ft
        if (posY > 5*12) corner = 1; //If we've gone forward 6 ft, find the corner
      }
      if (target_corner == 2){ //Target: 7ft, 7ft
        if (posX > 5*12) corner = 1; //If we've traveled to the first corner, and then right 6 ft, find the corner
      }
      if (target_corner == 3){ //Target: 7ft, 1ft
        if (posY < 2*12 && posX > 5*12) corner = 1; //If we're within 1 ft of our original y position, find the corner
      }
    }
    getMins();
    if (finding_state) findWall();
    else if (!corner) navigate(); //Hug the left wall
    else findCorner();

    Serial.print("Position: "); Serial.print(posX); Serial.print(", "); Serial.println(posY);
    Serial.print("Orientation: "); Serial.println(orientation * 180 / pi);
    Serial.print("Total left coutner: "); Serial.println(totalCounterL);
    Serial.print("Total right counter:"); Serial.println(totalCounterR);
  }

  else if (!finished){
    scan_range(0, 180, divisions); // This is where the most time is spent. Gathering obstacle data.
    get_position(); // Calculate our current position.
    navigate();
    if (posX < 2*12 && posY < 2*12){ // Might adjust this later. Front wheel needs to be within a 1.5ft radius of the starting region
      stop();
      noInterrupts();
      finished = 1;
    }
  }
}