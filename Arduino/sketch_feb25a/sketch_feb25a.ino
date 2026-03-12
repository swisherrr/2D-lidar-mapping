// ---- Pins to driver IN1-IN4 ----
int pins[4] = {10, 11, 13, 12};   // change if your wiring is different

// 8-step half-step sequence
int sequence[8][4] = {
  {1,0,0,1},
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1}
};

int STEP_DELAY = 1;   // increase if it shakes

void setup() {

  for(int i=0;i<4;i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }

  stepMotor(-50);
  delay(500);
  stepMotor(50);
  delay(500);
  stepMotor(50);
  delay(500);
  stepMotor(-50);
  delay(500);
  stopMotor();

  
  // stepMotor(-25);   // move LEFT 50
  // delay(1000);

  // stepMotor(-25);    // move RIGHT 25 (back to zero)
  // delay(1000);

  // stopMotor();      // turn coils off so it doesn't get hot
}

void loop() {

}

// ---- Move motor ----
void stepMotor(int steps){

  int dir = (steps > 0) ? 1 : -1; // direction
  steps = abs(steps);              // total steps

  for(int s=0; s<steps; s++){

    if(dir > 0){
      for(int k=0;k<8;k++){
        writeStep(k);
      }
    } else {
      for(int k=8;k>0;k--){
        writeStep(k);
      }
    }
  }
}

// ---- Apply one step ----
void writeStep(int idx){
  for(int i=0;i<4;i++){
    digitalWrite(pins[i], sequence[idx][i]);
  }
  delay(STEP_DELAY);
}

// ---- Turn motor off ----
void stopMotor(){
  for(int i=0;i<4;i++){
    digitalWrite(pins[i], LOW);
  }
}

