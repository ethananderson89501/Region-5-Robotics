#include <Servo.h>
#include <PinChangeInterrupt.h>

// I/O pins
#define SERVO_PIN 9

#define ECHO 8
#define TRIG 7

//Currently the components are on the rover backward, hence IN2/IN1 and IN4/IN3 are switched
#define IN1 11
#define IN2 3 
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

//Navigation states
int corner = 0;
int corners_checked = 0;

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
double inches_per_tick = 0.29;
//double radians_per_tick = 0.0205;
//double inches_forward_per_tick = 0.0440;
//double inches_sideways_per_tick = 0.0912;
double radians_per_tick = 0.0199;
//double inches_sideways_per_radian = 4.3743;
//double inches_forward_per_radian = 2.1155;
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
unsigned long last_time1;
unsigned long last_time2;
int no_print;

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
      
      else if (backingOut && center_min >= 7) goForward();
      else if (backingOut && left_min < 10){
        backingOut = 0;
        goForward();
      }
      
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

  // Adjust for bad values. Values should be a single digit positive integer.
  /*
  if (currentCounterL > 10){
    if (state == forward || state == reverse) currentCounterL = currentCounterR;
    if (state == left) currentCounterL = currentCounterR / 2;
    if (state == right) currentCounterL = currentCounterR * 2;
  }
  if (currentCounterR > 10){
    if (state == forward || state == reverse) currentCounterR = currentCounterL;
    if (state == left) currentCounterR = currentCounterL / 2;
    if (state == right) currentCounterR = currentCounterL * 2;
  }
*/
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
    double dx = turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    double dy = turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);
    posX = posX + dx;
    posY = posY + dy;
    orientation += dori;
  }
  else if (state == right){
    double dori = -1 * (currentCounterL + currentCounterR) * radians_per_tick;
    double dx = turning_radius * cos(dori + orientation)  - turning_radius * cos(orientation);
    double dy = turning_radius * sin(dori + orientation) - turning_radius * sin(orientation);
    posX = posX + dx;
    posY = posY + dy;
    orientation += dori;
  }
  
  if (orientation > pi) orientation -= 2*pi;

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
  //Temp sensing
  
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

  if (!corner) navigate(); //Hug the left wall

  ////////////// Navigation testing logic //////////////////////////
  /*
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
  

  if (elapsed_time2 > 1000000){
    if (state == forward) turnLeft();
    else if (state == left) goReverse();
    else if (state == reverse) turnRight();
    else if (state == right) goForward();
    last_time2 = current_time;
  }
  */
}
