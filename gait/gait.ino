#include "motor_ibt.h"

/* Improve ADC sampling rate
   http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html
   https://www.instructables.com/id/Girino-Fast-Arduino-Oscilloscope/
*/
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/*DEBUGGING PURPOSES*/
#define DEBUG 1

#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

// constructor 
Motor_IBT LeftHip(11, 12, A0, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT LeftKnee(2, 3, A1, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT RightHip(7, 8, A2, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT RightKnee(9, 10, A3, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  //----------- Set PWM frequency for D11 & D12 --------------
  //  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz

  //----------- Set PWM frequency for D2, D3 & D5 ------------
  //  TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz
  TCCR3B = TCCR3B & B11111000 | B00000100; // set timer 3 divisor to 256 for PWM frequency of 122.55 Hz

  //----------- Set PWM frequency for D6, D7 & D8 ------------
  TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz

  //----------- Set PWM frequency for D9 & D10 ---------------
  TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //set adc prescaler to 16
  sbi(ADCSRA, ADPS2); // set bit
  cbi(ADCSRA, ADPS1); // clear bit
  cbi(ADCSRA, ADPS0);
}

void loop()
{
  long int t = micros();
  LeftHip.FilterMovADC(630, 880, 45, -15);
  //  Serial.print(LeftHip.GetAngle());
  dprint(LeftHip.GetAngle());
  Serial.println(micros() - t);

  //    trajectori(45, -45, 45, -45);
  //    trajectori(45,0,45,0);
  //    trajectori(-10,0, -10,0);

  //      gaitLeft(45, -45);
  //      gaitLeft(45, 0);
  //      gaitLeft(-10, 0);

  //    gaitRight(45, -45);
  //    gaitRight(45, 0);
  //    gaitRight(-10, 0);

  //      int angle_2 = 45;
  //      LeftKnee.FilterMovADC(290, 540, 45, -15);
  //      LeftKnee.GoToAngle(angle_2, 0, 15, 15, 35, 30, false); //15, 15, 40, 35, // 40, 40, 50, 30,
  //      LeftKnee.Driver(LeftKnee.GetRotate(), false, LeftKnee.GetSpeed());

  //    int angle_1 = 0;
  //    LeftHip.FilterMedADC(630, 880, 45, -15);
  //    LeftHip.GoToAngle(angle_1, LeftKnee.GetAngle(), 100, 90, 60, 35); // 150 100 60 35
  //    LeftHip.Driver(LeftHip.GetRotate(), true, LeftHip.GetSpeed());

  //      int angle_3 =-45;
  //      RightKnee.FilterMedADC(320, 570, 45, -15);
  //      RightKnee.GoToAngle(angle_3, 0, 15, 15, 40, 35, false);
  //      RightKnee.Driver(RightKnee.GetRotate(), false, RightKnee.GetSpeed());

  //  int angle_4 = 0;
  //  RightHip.FilterMedADC(600, 850, 45, -15);
  //  RightHip.GoToAngle(angle_4, RightKnee.GetAngle(), 150, 100, 60, 35, true);
  //  RightHip.Driver(RightHip.GetRotate(), true, RightHip.GetSpeed());
}

void gaitLeft (double angle_1, double angle_2) {
  do {
    LeftHip.FilterMovADC(630, 880, 45, -15);
    LeftKnee.FilterMovADC(290, 540, 45, -15);
    LeftHip.GoToAngle(angle_1, LeftKnee.GetAngle(), 100, 90, 60, 35, true);
    LeftHip.Driver(LeftHip.GetRotate(), true, LeftHip.GetSpeed());
    LeftKnee.GoToAngle(angle_2, 0, 15, 15, 35, 30, false);
    LeftKnee.Driver(LeftKnee.GetRotate(), false, LeftKnee.GetSpeed());
  } while (LeftHip.GetRotate() != STOP || LeftKnee.GetRotate() != STOP );
}

void gaitRight(double angle_1, double angle_2) {
  do {
    RightHip.FilterMedADC(600, 850, 45, -15);
    RightKnee.FilterMedADC(320, 570, 45, -15);
    RightHip.GoToAngle(angle_1, RightKnee.GetAngle(), 150, 100, 60, 35, true);
    RightKnee.GoToAngle(angle_2, 0, 15, 15, 40, 35, false);
    RightHip.Driver(RightHip.GetRotate(), true, RightHip.GetSpeed());
    RightKnee.Driver(RightKnee.GetRotate(), false, RightKnee.GetSpeed());
  } while (RightHip.GetRotate() != STOP || RightKnee.GetRotate() != STOP );
}


void trajectori(double angle_1, double angle_2, double angle_3, double angle_4) {
  do {
    // adc filter
    LeftHip.FilterMedADC(630, 880, 45, -15);
    RightHip.FilterMedADC(600, 850, 45, -15);
    LeftKnee.FilterMedADC(290, 540, 45, -15);
    RightKnee.FilterMedADC(320, 570, 45, -15);

    // go to angle
    LeftHip.GoToAngle(angle_1, LeftKnee.GetAngle(), 100, 90, 60, 35, true);
    LeftHip.Driver(LeftHip.GetRotate(), true, LeftHip.GetSpeed());

    RightHip.GoToAngle(angle_3, RightKnee.GetAngle(), 150, 100, 60, 35, true);
    RightHip.Driver(RightHip.GetRotate(), true, RightHip.GetSpeed());

    LeftKnee.GoToAngle(angle_2, 0, 15, 15, 35, 30, false);
    LeftKnee.Driver(LeftKnee.GetRotate(), false, LeftKnee.GetSpeed());

    RightKnee.GoToAngle(angle_4, 0, 15, 15, 40, 35, false);
    RightKnee.Driver(RightKnee.GetRotate(), false, RightKnee.GetSpeed());
  } while (LeftHip.GetRotate() != STOP || LeftKnee.GetRotate() != STOP || RightHip.GetRotate() != STOP || RightKnee.GetRotate() != STOP );
}
