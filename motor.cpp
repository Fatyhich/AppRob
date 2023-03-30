#include "motor.h"

void motor::init(int _FE, int _SE, int _out, int _dir) {
  timer = millis();

  E1 = _FE;
  E2 = _SE;
  out = _out;
  dir = _dir;

  F = 0;
  S = 0;

  flagInt = false;
  count = 0;

  pinMode(E1, INPUT_PULLUP);
  pinMode(E2, INPUT_PULLUP);
  pinMode(out, OUTPUT);
  pinMode(dir, OUTPUT);

  Send2Driver(0);
}

void motor::interruptListener() {
  flagInt = true;
}

void motor::tick() {
  if (flagInt) {
    F = digitalRead(E1);
    S = digitalRead(E2);

    if (F == S)
      count++;
    else
      count--;

    flagInt = false;
  }
  calcRealVel();
}

void motor::calcRealVel() {
  if (millis() - timer > RATE) {
    timer = millis();
    int delta = count - countPrev;
    countPrev = count;
    realVel = delta * 0.0285 * (1000 / RATE);

    // Serial.print(PWM);
    // Serial.print(",");
    // Serial.println(realVel);
  }
}

void motor::VelPID() {
  if (millis() - timer > RATE) {
    timer = millis();
    double errVel = goalVel - realVel;
    double sum = ConsPWM(sum + errVel * kI * (float)(1000 / RATE));
    PWM = ConsPWM(kP * errVel + sum + kD * errVel);
    analogWrite(out, PWM);

    // Serial.println(PWM);
  }
}

double motor::ConsPWM(double ch) {
  if (ch > MAX_PWM)
    return MAX_PWM;
  else if (ch < MIN_PWM)
    return MIN_PWM;
  else if (MIN_PWM < ch && ch < MAX_PWM)
    return ch;
}

void motor::Send2Driver(double _PWM) {
  if (_PWM >= 0)  // по часам
  {
    analogWrite(out, _PWM);
    digitalWrite(dir, LOW);
  } else  // против часов
  {
    analogWrite(out, 255 + _PWM);
    digitalWrite(dir, HIGH);
  }
}
