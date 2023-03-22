#define out 9
#define in_1 6
#define in_2 7
#define FE 2
#define SE 3
#define RATE 20

float kP = 0.75;
float kI = 1;
float Sum = 0;
#define kD 0

#include "GyverFilters.h"

GKalman Filter (5, 0.02);

bool flag = false;
int val = 0;
int valPrev = 0;
int Delta = 0;
double ErrVel = 0;
double RealRadVel = 0;
double tempRealRadVel = 0;

int PWM = 0;
int F = 0;
int S = 0;
int GoalVel = 30;

long Timer = 0;

void Flag() {
  flag = true;
}

void tick() {

  if (flag) {

    F = digitalRead(FE);
    S = digitalRead(SE);

    if (F == S) val++;
    else val--;

    flag = false;
  }
}

void CalcRadVel() {
  if (millis() - Timer > RATE) {
    Timer = millis();
    Delta = val - valPrev;
    valPrev = val;
    tempRealRadVel = Delta * 0.0285 * (1000 / RATE);
    RealRadVel = Filter.filtered(tempRealRadVel);

    Serial.print(GoalVel);
    Serial.print(",");
    Serial.println(RealRadVel);
  }
}

void velPID() {
  if (millis() - Timer > RATE) {
    Timer = millis();
    ErrVel = GoalVel - RealRadVel;
    Sum = consPWM(Sum + ErrVel * kI * (float)(1000 / RATE));
    PWM = (int)consPWM(kP * ErrVel + Sum + kD *  ErrVel);
    analogWrite(out, PWM);

    //Serial.println(PWM);
  }
}

float consPWM(int inc) {
  if (inc > 0) {
    if (inc > 255)
      return 250;
    else if (inc < 25)
      return 25;
    else return inc;
  }
  else if (inc < 0) {
    if (inc > -255)
      return abs(inc);
    else return 0;
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("GoalVel, RealVel");
  attachInterrupt(0, Flag, CHANGE);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(in_1, LOW);
  digitalWrite(in_2, HIGH);
  analogWrite(out, 0);

  Timer = millis();
}

void loop() {
  tick();
  CalcRadVel();
 // velPID();

  if (Serial.available() > 0) {
    kI = Serial.parseInt();
  }
}
