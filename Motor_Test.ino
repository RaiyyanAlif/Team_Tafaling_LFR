#define AIN1 2
#define AIN2 4
#define BIN1 6
#define BIN2 7
#define SLP 13

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SLP, OUTPUT);

  digitalWrite(SLP, HIGH);   // Enable DRV8833
}

void forward() {
  analogWrite(AIN1, 200);
  analogWrite(AIN2, 0);

  analogWrite(BIN1, 200);
  analogWrite(BIN2, 0);
}

void backward() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 200);

  analogWrite(BIN1, 0);
  analogWrite(BIN2, 200);
}

void stopMotor() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

void loop() {
  forward();
  delay(5000);

  backward();
  delay(5000);
}
