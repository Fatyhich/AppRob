#include "motorConfig.h"
#include "Arduino.h"

class motor
{
private:
  unsigned int E1;
  unsigned int E2;
  unsigned int out; // ШИМ пин
  unsigned int dir; // направление вращения

  bool F; // считывание состояний Датчика Холла
  bool S;

  bool flagInt; // флаг прерывания

  long timer;

  int count;   // кол-во tick-ов
  int countPrev;
  int PWM;     // ШИМ
  int goalVel; // параметры скорости
 
  double realVel;
  double realVelPrev;

  void calcRealVel();
  void VelPID();
  

  double ConsPWM(double ch);

public:
  void init(int _FE, int _SE, int _Out, int _Dir);
  void interruptListener();
  void tick();
  void Send2Driver(double _PWM);
};
