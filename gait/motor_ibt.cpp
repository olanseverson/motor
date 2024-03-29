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

  _filteredADC = analogRead(_SensorPin);
  _angleTolerance = 1;

  // initialize variables
  for (int i = 0; i < MA_COEFF; i++)
  {
    BUFFER[i] = _filteredADC;
  }
  _idxBuff = 0;
  _PID_d = 0.0;
  _PID_i = 0.0;
  _prev_error = 0.0;
  _PID_value = 0.0;
}

void Motor_IBT::Driver(rotateState IsRotate, bool isHip, int Speed, int DownCCW, int DownCW)
{
  if (IsRotate == CCW) {
    int turun = Speed; // naik ke belakang
    if (isHip && _angle >= 0) {
      turun = DownCCW - _angle / 2; // jatuh
    }
    analogWrite(_LPWM, 0);
    analogWrite(_RPWM, abs(turun));
  } else if (IsRotate == CW) {
    int gerak = Speed; // naik ke depan
    if (isHip && _angle < 0) {
      gerak = DownCW + _angle / 2; // jatuh
    }
    analogWrite(_LPWM, abs(gerak));
    analogWrite(_RPWM, 0);
  } else if (IsRotate == STOP) {
    analogWrite(_LPWM, 255);
    analogWrite(_RPWM, 255);
  }
}


void Motor_IBT::FilterMovADC(int lowADC, int highADC, int highAngle, int lowAngle)
{
  float ir_val[MA_COEFF];
  for (int i = 0; i < MA_COEFF; i++) {
    ir_val[i] = analogRead(_SensorPin);
  }

  int len = MA_COEFF;
  long temp = 0;
  for (int i = 0; i < len; i++) {
    temp += ir_val[i];
  }
  _filteredADC = temp / MA_COEFF;
  _angle = map(_filteredADC, lowADC, highADC, highAngle, lowAngle);
}

void Motor_IBT::FilterMedADC(int lowADC, int highADC, int highAngle, int lowAngle)
{
  float ir_val[MED_COEFF];
  for (int i = 0; i < MED_COEFF; i++) {
    ir_val[i] = analogRead(_SensorPin);
  }

  int len = MED_COEFF;
  for (int i = 0; i < (len - 1); i++) {
    bool flag = true;
    for (int o = 0; o < (len - (i + 1)); o++) {
      if (ir_val[o] > ir_val[o + 1]) {
        int t = ir_val[o];
        ir_val[o] = ir_val[o + 1];
        ir_val[o + 1] = t;
        flag = false;
      }
    }
    if (flag) break;
  }
  _filteredADC = ir_val[len / 2];
  _angle = map(_filteredADC, lowADC, highADC, highAngle, lowAngle);
}


void Motor_IBT::GoToAngle(int toAngle, int addedTorque, int cForward, int cBackward, int bias1, int bias2, bool isHip)
{
  double Penambah = 0;
  _target = toAngle;
  double delta = _target - _angle;
  _PID_i = _PID_i + delta;
  _PID_d = delta - _prev_error;
  _prev_error = delta;
//  _PID_value = (delta * 0.8 + _PID_i * 0.06 + _PID_d * 0.8) * abs(sin((_angle / 180) * 3.14));

  // Penambah berfungsi untuk memberikan tambahan pwm ketika terjadi
  if (isHip) {
    _PID_value = (delta * 0.8 + _PID_i * 0.06 + _PID_d * 0.8) * abs(sin((_angle / 180) * 3.14));
    if ((_angle < 0) && _target < 0) //
      Penambah = cBackward * abs(sin((_angle / 180) * 3.14)) + (50 * abs(sin((addedTorque / 180) * 3.14))); // naik ke belakang
    else if ((_angle > 0) && _target > 0)
      Penambah = cForward * abs(sin((_angle / 180) * 3.14)) + (10 * abs(sin((addedTorque / 180) * 3.14))); //naik ke depan
  } else {
    _PID_value = (delta * 1.5 + _PID_i * 2.2 + _PID_d * 2.1) * abs(sin((_angle / 180) * 3.14)); // 1.5,1.8,2.1
    Penambah = cForward * abs(sin((_angle / 180) * 3.14));
  }


  // batasi dalam range 0-bias1
  if (Penambah < 0)Penambah = 0;
  if (Penambah > bias1)Penambah = bias1;

  //  batasi speed dalam range :   bias2+penambah< pid < bias1+penambah
  if (abs(_PID_value) > (bias1 + Penambah)) {
//    dshow("1_");
    _speed = bias1 + Penambah;
  }
  else if (abs(_PID_value) < (bias2 + Penambah) ) {
//    dshow("2_");
    _speed = bias2 + Penambah;
  }
  else {
//    dshow("3_");
    _speed = abs(_PID_value);
  }
  //      String buf = " posisi pid penambah speed " ;
  //      buf = buf + String(_angle) + " " + String(_PID_value) + " " + String(Penambah) + " " + _speed;
  //      Serial.println(buf);
  //      dshow(buf);
//      dprint(_angle);dprint(_target); dprint(_PID_value); dprint(Penambah);dprint(_speed);

  if (abs(delta) > _angleTolerance) { // di luar toleransi error
    _isRotate = CCW;
    if (delta > 0) {
      _isRotate = CW;
    }
  } else { // error dapat ditoleransi
    _isRotate = STOP;
    _PID_i = 0;
    _prev_error = 0;
    _PID_value = 0;
  }
}

void Motor_IBT::ResetPID() {
  _isRotate = STOP;
  _PID_i = 0;
  _prev_error = 0;
  _PID_value = 0;
}
/*
  || @changelog
  || | 1.0 2019-03-13 - Yoland Nababan : Initial Release
  || #
*/
