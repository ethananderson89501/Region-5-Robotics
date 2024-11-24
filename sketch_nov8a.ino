#include <Servo.h>
#include <string.h>
#include <Wire.h>
// #include <LiquidCrystal_I2C.h>
#define SERVO_PIN 3
#define LPT 2

#define ECHO 8
#define TRIG 7

#define IN1  12    //Right motor(K1/K2) direction Pin 7
#define IN2  11    //Right motor(K1/K2) direction Pin 8
#define IN3  9    //Left motor(K3/K4) direction Pin 9
#define IN4  10   //Left motor(K3/K4) direction Pin 10
#define ENA  5    //D5 to ENA PWM speed pin for Right motor(K1/K2)
#define ENB  6    //D6 to ENB PWM speed pin for Left motor(K3/K4)

#define FAST_SPEED  150 
#define SPEED  255   
#define TURN_SPEED  150
#define BACK_SPEED1  100
#define BACK_SPEED2  90 

// LiquidCrystal_I2C lcd(0x27,16,2);

int divisions = 3;

Servo head;

void go_Advance()  //motor rotate clockwise -->robot go ahead
{
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN2,LOW );
  digitalWrite(IN1,HIGH);
  set_Motorspeed(250, 250);
}
void go_Back() //motor rotate counterclockwise -->robot go back
{
  digitalWrite(IN4,HIGH);
  digitalWrite(IN3,LOW); 
  digitalWrite(IN2,HIGH);
  digitalWrite(IN1,LOW);
}
void stop_Stop() //motor brake -->robot stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,LOW); 
  set_Motorspeed(0,0);
}
void go_Right()  //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN1,LOW);
  set_Motorspeed(SPEED,SPEED);
  
}
void go_Left() //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  digitalWrite(IN4,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN1,HIGH);
  set_Motorspeed(SPEED,SPEED);
 
}
/*set motor speed */
void set_Motorspeed(int lspeed,int rspeed) //change motor speed
{
  analogWrite(ENB,lspeed);//lspeed:0-255
  analogWrite(ENA,rspeed);//rspeed:0-255   
}

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);

  head.attach(SERVO_PIN);
  head.write(90);
  delay(2000);
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  pinMode(ENA, OUTPUT);  
  pinMode(ENB, OUTPUT);

  stop_Stop();

  // lcd.init();
  // lcd.backlight();

}

double * scan_range(double theta_0, double theta_1, int divisions){
  double *dists = new double[divisions + 1];
  double delta = (theta_1 - theta_0) / divisions;
  for (int i = 0; i <= divisions; i++){
    double angle = theta_0 + i * delta; // 2 * theta_1 since divisions is 2 * on second pass
    head.write(angle);
    delay(500); // wait to get there
    double dist = watch();
    Serial.println(dist);
    delay(500);
    dists[i] = dist;
    // Serial.println(angle);
  }
  return dists;
}

double watch(){
  digitalWrite(TRIG,LOW);
  delayMicroseconds(5);                                                                            
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10); // give or take 7 feet
  digitalWrite(TRIG,LOW);
  long echo_time=pulseIn(ECHO,HIGH);
  double echo_distance=echo_time / 148.0; //how far away is the object in inches
  return echo_distance;
}

void loop() {
  int divisions = 2;
  double * dists = scan_range(45, 135, divisions);
  int length = divisions * 2 + 1;
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
  Serial.println("]");
  if (straight >= 10){
    go_Advance();
  }else if (left_sum > right_sum){
    go_Left();
  }else{
    go_Right();
  }
  // go_Advance();
  delay(2000);
  stop_Stop();
}
