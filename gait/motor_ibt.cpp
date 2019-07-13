/*
  ||
  || @file motor_ibt.cpp
  || @version 1.0
  || @author Yoland Nababan
  ||
  || @description
  || | Drive motor using BTS7960
  || | https://electronics.stackexchange.com/questions/398556/how-to-control-a-motor-driver-bts7960-without-pwm
  || #
  ||
  ||
*/
#include "motor_ibt.h"

//Constructor
Motor_IBT::Motor_IBT(int Pin_RPWM, int Pin_LPWM, int SensorPin, Stream &serial):
  _RPWM(Pin_RPWM),
  _LPWM(Pin_LPWM),
  _SensorPin(SensorPin),
  serial(serial)
{
  pinMode(_RPWM, OUTPUT);
  pinMode(_LPWM, OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000101; // reduce PWM frequency
  
  _filteredADC = analogRead(_SensorPin);
  _angleTolerance = 2;

  // initialize variables
  for (int i = 0; i < MA_COEFF; i++)
  {
    BUFFER[i] = _filteredADC;
  }
  _idxBuff = 0;
  _pid_d = 0.0;
  _pid_i = 0.0;
  _prev_error = 0.0;
  _pid_value = 0.0;
}

void Motor_IBT::Driver(rotateState IsRotate, int Speed = 255)
{
  switch (IsRotate)
  {
    case CCW :
      int turun;
      if (_filteredADC < 0)
        turun = 25 - 2 * _filteredADC;
      else
        turun = 20;
      analogWrite(_LPWM, 0);
      analogWrite(_RPWM, turun);
      break;
    case CW :
      analogWrite(_LPWM, Speed);
      analogWrite(_RPWM, 0);
      break;
    case STOP :
      analogWrite(_LPWM, Speed);
      analogWrite(_RPWM, Speed);
      break;
  }
}


void Motor_IBT::FilterMovADC()
{
  this->_ADC = analogRead(_SensorPin);
  BUFFER[_idxBuff] = _ADC;
  _idxBuff++;
  if (_idxBuff == MA_COEFF)
  {
    _idxBuff = 0;
  }
  long int temp = 0;
  for (int i = 0; i < MA_COEFF; i++)
  {
    temp += BUFFER[i];
  }
  _filteredADC = temp / MA_COEFF;
  _angle = map(_filteredADC, 0, 1023, 0, 300);
}

void Motor_IBT::FilterMedADC()
{
  float ir_val[MED_COEFF];
  for (int i = 0; i < MED_COEFF; i++) {
    ir_val[i] = analogRead(_SensorPin);
  }

  int size = MED_COEFF;
  for (int i = 0; i < (size - 1); i++) {
    bool flag = true;
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (ir_val[o] > ir_val[o + 1]) {
        int t = ir_val[o];
        ir_val[o] = ir_val[o + 1];
        ir_val[o + 1] = t;
        flag = false;
      }
    }
    if (flag) break;
  }
  _filteredADC = ir_val[size / 2];
  _angle = map(_filteredADC, 0, 1023, 0, 300);
}


void Motor_IBT::GoToAngle(int toAngle, int Speed)
// Speed   : 0 - 255
// toAngle : 0 - 1023 degree
{
  int delta = toAngle - _filteredADC;
  _target = toAngle;
  _speed = Speed;

  if (abs(delta) > GetTolerance()) {
    _IsRotate = CW;
    if (delta > 0) {
      _IsRotate = CCW;
    }
  }
  else
  { // stop
    _IsRotate = STOP;
    _pid_i = 0;
    _prev_error = 0;
    _pid_value = 0;
  }
}

int Motor_IBT::PositionPID (double Kp, double Ki , double Kd)
{
  double delta = _target - _filteredADC;
  _pid_i = _pid_i + delta;
  _pid_d = delta - _prev_error;
  _prev_error = delta;
  _pid_value = (delta * Kp + _pid_i * Ki + _pid_d * Kd) * abs(sin((_filteredADC / 180) * 3.14));
  double Penambah = 10 * sin((_filteredADC / 180) * 3.14);
  if (Penambah < 0)
    Penambah = 0;
  if (Penambah > 50)
    Penambah = 50;

  int pwm = 0;
  if (abs(_pid_value) > (50 + Penambah))
    pwm = 50 + Penambah;
  else if (abs(_pid_value) < (20 + Penambah) )
    pwm = 20 + Penambah;
  else
    pwm = _pid_value;

  return pwm;
}


/*
  || @changelog
  || | 1.0 2019-03-13 - Yoland Nababan : Initial Release
  || #
*/
