volatile int counter = 0;
int aLastState;

void count() {
  int aState = digitalRead(3);
  int bState = digitalRead(4);

  if (aState != aLastState) {  // Only process on change
    if (bState != aState) counter++;
    else counter--;
  }
  aLastState = aState;
}

void setup() {
  pinMode(3, INPUT_PULLUP); // A
  pinMode(4, INPUT_PULLUP); // B
  Serial.begin(9600);

  aLastState = digitalRead(3);
  attachInterrupt(digitalPinToInterrupt(3), count, CHANGE);
}

void loop() {
  noInterrupts();  // Prevent race conditions
  int countCopy = counter;
  interrupts();

  Serial.println(countCopy);
  delay(100);
}
