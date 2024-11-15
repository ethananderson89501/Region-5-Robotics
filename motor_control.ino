#include <Servo.h>
#include <string.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define SERVO_PIN 3
#define LPT 2

#define ECHO 8
#define TRIG 7

LiquidCrystal_I2C lcd(0x27,16,2);

int divisions = 18;

Servo head;


void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);

  head.attach(SERVO_PIN);
  head.write(90);
  delay(2000);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

}

double * scan_range(double theta_0, double theta_1, int divisions){
  double *dists = new double[divisions * 2];
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions * 2; i++){
    double angle = i < divisions ? delta * i : 360 - delta * i;
    head.write(angle);
    delay(150); // wait to get there
    double dist = watch();
    dists[i] = dist;
  }
  return dists;
}

double watch(){
  long echo_distance;
  digitalWrite(TRIG,LOW);
  delayMicroseconds(5);                                                                            
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10); // give or take 7 feet
  digitalWrite(TRIG,LOW);
  echo_distance=pulseIn(ECHO,HIGH);
  echo_distance=echo_distance / 148.0; //how far away is the object in inches
  return echo_distance;
}

void loop() {
  scan_range(0, 180, divisions);

}

// Pin assignments
int ENA = 4; // Enable pin for motor A
int IN1 = 5; // IN1 pin on H-bridge
int IN2 = 6; // IN2 pin on H-bridge
int IN3 = 10; // IN3 pin on H-bridge
int IN4 = 11; // IN4 pin on H-bridge
int ENB = 12; // Enable pin for motor B

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
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

int duty_cycleA;
int duty_cycleB;

void loop() {

  // Motor forward
  forward();
  delay(2000);

  /*
  if (distanceCenter < 15){ // Obstacle Detected
    if (distanceRight < distanceLeft){
      turnLeft();
      delay(500); // Exact time TBD
    }
    else{
      turnRight();
      delay(500); // Exact time TBD
    }
  }
  */

  turnLeft();
  delay(2000);

  turnRight();
  delay(2000);

  reverse();
  delay(2000);
}

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
  duty_cycleA = 128;
  duty_cycleB = 192;
  analogWrite(IN1, duty_cycleA);
  analogWrite(IN2, 0);
  analogWrite(IN3, duty_cycleB);
  analogWrite(IN4, 0);
}

void turnRight(){
  duty_cycleA = 192;
  duty_cycleB = 128;1
  analogWrite(IN1, 0);
  analogWrite(IN2, duty_cycleA);
  analogWrite(IN3, 0);
  analogWrite(IN4, duty_cycleB);
}
