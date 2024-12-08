#include <Servo.h>
#include <string.h>
#include <Wire.h>

#define SERVO_PIN 9
//#define LPT 2

#define ECHO 8
#define TRIG 7

#define ENA 4 
#define IN1 5
#define IN2 6 
#define IN3 3 
#define IN4 11 
#define ENB 12

int divisions = 6;

Servo head;

int duty_cycleA;
int duty_cycleB;

void forward(){
  duty_cycleA = 128;
  duty_cycleB = 128;
  analogWrite(IN1, 0);
  analogWrite(IN2, duty_cycleA);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
}

void stop(){
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void reverse(){
  duty_cycleA = 128;
  duty_cycleB = 128;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, duty_cycleB);
}

void turnLeft(){
  duty_cycleA = 192;
  duty_cycleB = 128;
  analogWrite(IN1, 0);
  analogWrite(IN2, duty_cycleA);
  analogWrite(IN3, 0);
  analogWrite(IN4, duty_cycleB);
}

void turnRight(){
  duty_cycleA = 128;
  duty_cycleB = 192;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
}

double watch(){
  digitalWrite(TRIG,LOW);
  delayMicroseconds(5);                                                                            
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10); // give or take 7 feet
  digitalWrite(TRIG,LOW);
  long echo_time=pulseIn(ECHO,HIGH);
  Serial.print("Echo time: ");
  Serial.println(echo_time);
  double echo_distance=echo_time / 148.0; //how far away is the object in inches
  return echo_distance;
}

double * scan_range(double theta_0, double theta_1, int divisions){
  double *dists = new double[divisions + 1];
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions; i++){
    double angle = theta_0 + i * delta; // 2 * theta_1 since divisions is 2 * on second pass
    head.write(angle);
    if (i == 0) delay(300); // wait to get there
    else delay(30);
    double dist = watch();
    Serial.println(dist);
    delay(30);
    dists[i] = dist;
    //Serial.println(angle);
  }
  return dists;
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
  head.write(90);
  delay(2000);
  
  Serial.begin(9600);
  
  stop();
}

void loop() {
  double * dists = scan_range(0, 180, divisions);
  double length = divisions * 2 + 1;
  double left_sum = 0;
  double right_sum = 0;
  double straight;
  Serial.print("[");
  for (int i = 0; i <= divisions; i++){
    Serial.print(dists[i]);
    Serial.print(", ");
    if (divisions % 2 == 0 && i == divisions / 2){
      straight = dists[i];
    }else if ( i <= divisions / 2
    ){
      right_sum += dists[i];
    }else{
      left_sum += dists[i];
    }
  }
  double left_average = left_sum / (divisions / 2);
  Serial.println("]");
  if (straight >= 10 && left_average < 15){
    forward();
  }else if (left_average >= 15){
    turnLeft();
    Serial.print("Turning left");
  }else{
    turnRight();
    Serial.print("Turning right");
  }
  //delay(200);
  //stop();
  /*
  forward();
  delay(1000);
  turnLeft();
  delay(1000);
  turnRight();
  delay(1000);
  reverse();
  delay(1000);
  */
}
