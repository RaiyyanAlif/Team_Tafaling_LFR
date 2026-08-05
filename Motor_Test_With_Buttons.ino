#define AIN1 2
#define AIN2 4
#define BIN1 6
#define BIN2 7
#define SLP 13

#define BTN_START 10   // Forward
#define BTN_STOP  12   // Backward

const int SPEED = 200;

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SLP, OUTPUT);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  digitalWrite(SLP, HIGH);

  stopMotor();
}

void forward() {
  analogWrite(AIN1, SPEED);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, SPEED);
  analogWrite(BIN2, 0);
}

void backward() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, SPEED);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, SPEED);
}

void stopMotor() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

void loop() {

  // D10 pressed -> Forward
  if (digitalRead(BTN_START) == LOW) {
    forward();
    delay(200);                  // debounce
    while (digitalRead(BTN_START) == LOW);
  }

  // D12 pressed -> Backward
  if (digitalRead(BTN_STOP) == LOW) {
    backward();
    delay(200);                  // debounce
    while (digitalRead(BTN_STOP) == LOW);
  }
}
