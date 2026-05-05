// ---- Motor A (left) ----
const int ENA = 5;
const int IN1 = 50;
const int IN2 = 51;

// ---- Motor B (right) ----
const int ENB = 6;
const int IN3 = 52;
const int IN4 = 53;

int speedPct = 60;

int pwmFromPct(int pct) {
  pct = constrain(pct, 0, 100);
  return map(pct, 0, 100, 0, 255);
}

void motorA(int pwm, bool fwd) {
  if (pwm <= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    return;
  }
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW : HIGH);
  analogWrite(ENA, pwm);
}

void motorB(int pwm, bool fwd) {
  if (pwm <= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
    return;
  }
  digitalWrite(IN3, fwd ? HIGH : LOW);
  digitalWrite(IN4, fwd ? LOW : HIGH);
  analogWrite(ENB, pwm);
}

void stopAll() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void drive(char cmd) {
  int pwm = pwmFromPct(speedPct);

  switch (cmd) {
    case 'w':
      motorA(pwm, true);
      motorB(pwm, true);
      Serial.println("Forward");
      break;

    case 's':
      motorA(pwm, false);
      motorB(pwm, false);
      Serial.println("Reverse");
      break;

    case 'a':
      motorA(pwm, false);
      motorB(pwm, true);
      Serial.println("Left");
      break;

    case 'd':
      motorA(pwm, true);
      motorB(pwm, false);
      Serial.println("Right");
      break;

    case 'x':
      stopAll();
      Serial.println("Stop");
      break;
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  Serial.begin(115200);    // USB debug
  Serial1.begin(9600);     // Bluetooth (HC-05 default)

  stopAll();

  Serial.println("Bluetooth Ready");
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    c = tolower(c);

    Serial.print("Received: ");
    Serial.println(c);

    if (c >= '0' && c <= '9') {
      speedPct = (c == '0') ? 100 : (c - '0') * 10;
      Serial.print("Speed: ");
      Serial.println(speedPct);
      return;
    }

    if (c == 'q') {
      stopAll();
      Serial.println("Quit");
      while (true);
    }

    if (c=='w' || c=='a' || c=='s' || c=='d' || c=='x') {
      drive(c);
    }
  }
}