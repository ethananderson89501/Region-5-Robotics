
int input_pin = 11;
int prevsensor = 0; 
int sensor = 0;
float treadsep = 0.01; // In Meters
float distance = 0;

void setup() {
  Serial.begin(9600);
  pinMode(11, INPUT);
  
}

void loop() {
  sensor = digitalRead(11);
  if(sensor == 0 && prevsensor == 1){
    distance = distance + treadsep;
  }
  prevsensor = sensor;
}
