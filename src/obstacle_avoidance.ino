#include <Servo.h>
#include <string.h>
#include <Wire.h>

#define SERVO_PIN 9

#define ECHO 8
#define TRIG 7

#define ENA 4 
#define IN1 5
#define IN2 6 
#define IN3 3 
#define IN4 11 
#define ENB 12

#define stopped 0
#define forward 1
#define left 2
#define right 3
#define reverse 4

const int divisions = 8;

Servo head;

int duty_cycleA;
int duty_cycleB;

int state;
int backingOut;

int direction;

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
  //Serial.print("Echo time: ");
  //Serial.println(echo_time);
  double echo_distance=echo_time / 148.0; //how far away is the object in inches
  return echo_distance;
}

double dists[divisions + 1];
void scan_range(double theta_0, double theta_1, int divisions, int directions){
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions; i++){
    double angle = theta_0 + i * delta; // 2 * theta_1 since divisions is 2 * on second pass
    if (direction == 1) head.write(angle);
    else head.write(theta_1 - angle);
    delay(30);
    double dist = watch();
    if (dist < 0) dist = 0; // Filters out invalid values
    //Serial.println(dist);
    if (direction == 1) dists[i] = dist;
    else dists[divisions - i] = dist;
    //Serial.println(angle);
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

void setup() {
  // Set pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // Initialize motor state (motor off)
  digitalWrite(ENA, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  digitalWrite(ENB, HIGH);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  digitalWrite(TRIG, LOW);
  
  head.attach(SERVO_PIN);
  head.write(180); //Start in left position
  delay(200);
  
  Serial.begin(9600);

  backingOut = 0;
  direction = 0; // Start going from left to right
  
  stop();
  delay(200);
  goForward();
}

double center_array[divisions / 3];
double left_array[divisions / 3];
double right_array[divisions / 3];
double center_min;
double right_min;
double left_min;

void loop() {
  scan_range(0, 180, divisions, direction);
  /*
  Serial.print("[");
  for (int i = 0; i <= divisions; i++){
    Serial.print(dists[i]);
    Serial.print(", ");
  }
  Serial.println("]");
*/
  // Find minimum value in the center
  //Serial.print("Center array: [");
  for (int i = 0; i <= divisions / 3; i++){
    center_array[i] = dists[i + divisions / 3 + 1];
    //Serial.print(center_array[i]);
    //Serial.print(", ");
  }
  //Serial.println("]");
  center_min = getMin(center_array, divisions / 3 + 1);
  //Serial.print("Center minimum: ");
  //Serial.println(center_min);

  // Find minimum value on the left side
  //Serial.print("Left array: [");
  for (int i = 0; i <= divisions / 3; i++){
    left_array[i] = dists[i + divisions *2/3 + 1];
    //Serial.print(left_array[i]);
    //Serial.print(", ");
  }
  //Serial.println("]");
  left_min = getMin(left_array, divisions / 3 + 1);
  //Serial.print("Left minimum: ");
  //Serial.println(left_min);

  // Find minimum value on the right side
  //Serial.print("Right array: [");
  for (int i = 0; i <= divisions / 3; i++){
    right_array[i] = dists[i];
    //Serial.print(right_array[i]);
    //Serial.print(", ");
  }
  //Serial.println("]");
  right_min = getMin(right_array, divisions / 3 + 1);
  //Serial.print("Right minimum: ");
  //Serial.println(right_min);

  // Logic to hug the left wall
  //Serial.println(backingOut);
  if (state == forward){
      if (center_min >= 10 && left_min < 12 && left_min > 6){
        goForward();
      }else if (left_min >= 12){
        turnLeft();
      }else if (right_min >= 12){
        turnRight();
      }
      else{
        backingOut = 1;
        goReverse();
        Serial.println("Backing out");
      }
  }
  else if (state == right){
    if (center_min < 10){
      turnRight();
    }
    else {goForward();}
  }
  else if (state == left){
    if (left_min >= 12){
      turnLeft();
    }
    else{ goForward();}
  }
  
  else if (state == reverse){
    if (right_min >= 8) turnRight();
    else goReverse();
  }
  
}
