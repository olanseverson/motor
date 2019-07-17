/*
  ||
  || @file motor_ibt.h
  || @version 1.0
  || @author Yoland Nababan
  ||
  || @description
  || | Drive motor using BTS7960
  || | https://electronics.stackexchange.com/questions/398556/how-to-control-a-motor-driver-bts7960-without-pwm
  || #
  ||
*/
/*
  IBT-2 Motor Control Board driven by Arduino.
  Speed and direction controlled by a potentiometer attached to analog input 0.
  One side pin of the potentiometer (either one) to ground; the other side pin to +5V
  Connection to the IBT-2 board:
  IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
  IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
  IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
  IBT-2 pin 8 (GND) to Arduino GND
  IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/
/*DEBUGGING PURPOSES*/
#define DEBUG 1
#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

#ifndef MOTOR_IBT_H
#define MOTOR_IBT_H


#include <Arduino.h>

#define MA_COEFF 100 //Moving average coefficient
#define MED_COEFF 100 //Median filter coefficient

enum rotateState {CW, CCW, STOP}; // 0 1 2

// LPWM high, L IS high

class Motor_IBT
{
  private:
    volatile int _ADC;
    volatile int _filteredADC;
    int BUFFER[MA_COEFF];
    int _idxBuff;
    volatile int _speed;
    volatile int _target;
    int _RPWM;
    int _LPWM;
    int _SensorPin;
    volatile float _angle;
    int _angleTolerance;
    
    //PID constants
    double _PID_d;
    double _PID_i;
    double _prev_error;
    double _PID_value;


  public:
    //methods
    Motor_IBT(int Pin_RPWM, int Pin_LPWM, int SensorPin, Stream &serial);
    void Driver(enum rotateState, bool isHip, int Speed);
    void FilterMovADC(int lowADC, int highADC, int highAngle, int lowAngle); //moving average filter
    void FilterMedADC(int lowADC, int highADC, int highAngle, int lowAngle); //median filter 
    void GoToAngle(int toAngle, int addedTorque, int cForward, int cBackward, int bias1, int bias2, bool isHip);

    /*GETTER*/
    int GetADC() {return this->_ADC;}
    int GetFilteredADC() {return _filteredADC;}
    int GetTarget() {return _target;}
    rotateState GetRotate() {return _isRotate;}
    int GetSpeed() {return _speed;}
    int GetTolerance () {return _angleTolerance;}
    int GetPIDValue(){return _PID_value;}
    float GetAngle(){return _angle;}

    /*SETTER*/
    void SetRotate(rotateState Rotate) {_isRotate = Rotate;}
    void SetFilteredADC(int filteredADC){_filteredADC = filteredADC;}
    void SetSpeed(int Speed) {_speed = Speed;}
    void SetTolerance (int tolerance){_angleTolerance = tolerance;}
    void SetPIDValue(int pidvalue){_PID_value = pidvalue;}
    void SetAngle(float angle){_angle = angle;}

    
    // variable
    Stream &serial;
    volatile rotateState _isRotate = STOP;
};

#endif
/*
  || @changelog
  || | 1.0 2019-07-13 - Yoland Nababan : Initial Release
  || #
*/
