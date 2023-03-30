#include "motor.h"

#define INT 0  // номер прерывания
#define _FE 2
#define _SE 3
#define _out 9
#define _dir 10

motor left;

void Rise() {
  left.interruptListener();       // тики считываются корректно
}

void setup() {
  Serial.begin(9600);

  attachInterrupt(INT, Rise, CHANGE);

  left.init(_FE, _SE, _out, _dir);
}

void loop() {
  left.tick();
  if (Serial.available() > 0) {
    double temp = Serial.parseInt();
    left.Send2Driver(temp);
  }
}
