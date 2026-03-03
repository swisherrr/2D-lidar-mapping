/*
  Arduino Mega + L298N (2 DC motors) WASD TEST over Serial

  Serial Monitor settings:
    Baud: 115200
    Line ending: No line ending

  Keys:
    w forward
    s reverse
    a left  (A rev, B fwd)
    d right (A fwd, B rev)
    x stop
    1-9 speed 10-90, 0=100
    q stop + print quit message
*/

// ---- Motor A (left) ----
const int ENA = 5;   // PWM
const int IN1 = 50;
const int IN2 = 51;

// ---- Motor B (right) ----
const int ENB = 6;   // PWM
const int IN3 = 52;
const int IN4 = 53;

int speedPct = 60; // 0-100

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
  digitalWrite(IN2, fwd ? LOW  : HIGH);
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
  digitalWrite(IN4, fwd ? LOW  : HIGH);
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
    case 'w': // forward
      motorA(pwm, true);
      motorB(pwm, true);
      Serial.println("CMD: W (forward)");
      break;

    case 's': // reverse
      motorA(pwm, false);
      motorB(pwm, false);
      Serial.println("CMD: S (reverse)");
      break;

    case 'a': // left turn
      motorA(pwm, false);
      motorB(pwm, true);
      Serial.println("CMD: A (left turn)");
      break;

    case 'd': // right turn
      motorA(pwm, true);
      motorB(pwm, false);
      Serial.println("CMD: D (right turn)");
      break;

    case 'x': // stop
      stopAll();
      Serial.println("CMD: X (stop)");
      break;
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  Serial.begin(115200);
  stopAll();

  Serial.println("=== L298N WASD TEST (2 motors) ===");
  Serial.println("Serial Monitor: 115200 baud, No line ending");
  Serial.println("WASD move | X stop | 1-9 speed | 0=100 | Q quit");
  Serial.print("Speed = "); Serial.print(speedPct); Serial.println("%");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    c = tolower(c);

    // speed keys
    if (c >= '0' && c <= '9') {
      speedPct = (c == '0') ? 100 : (c - '0') * 10;
      Serial.print("Speed set to ");
      Serial.print(speedPct);
      Serial.println("%");
      return;
    }

    if (c == 'q') {
      stopAll();
      Serial.println("Quit/Stop.");
      while (true) { delay(1000); } // halt
    }

    if (c=='w' || c=='a' || c=='s' || c=='d' || c=='x') {
      drive(c);
    }
  }
}