#include <Arduino_FreeRTOS.h>
#include <timers.h>
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

// constructor
Motor_IBT RightHip(7, 8, A2, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT LeftHip(11, 12, A0, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT RightKnee(9, 10, A3, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
Motor_IBT LeftKnee(2, 3, A1, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);

void TaskRH( void *pvParameters );
void TaskLH( void *pvParameters );
void TaskRK( void *pvParameters );
void TaskLK( void *pvParameters );

double angleRH = 0; double angleRK = -5; double angleLH = 20; double angleLK = -30;
//double angleRH = 0; double angleRK = 0; double angleLH = 0; double angleLK = 0;
//double angleRH = 20; double angleRK = -30; double angleLH = 0; double angleLK = -5;

int tDelay = 2;
/********************************************VOID SETUP***********************************************/
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

  //  set adc prescaler to 16
  sbi(ADCSRA, ADPS2); // set bit
  cbi(ADCSRA, ADPS1); // clear bit
  cbi(ADCSRA, ADPS0);

  // Now set up many tasks to run independently.
  xTaskCreate(TaskRH,  (const portCHAR *)"RightHIP"  ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskLH,  (const portCHAR *)"LeftHIP"   ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskRK,  (const portCHAR *)"RightKNEE" ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskLK,  (const portCHAR *)"LeftKNEE"  ,  1000,  NULL,  1,  NULL );
  vTaskStartScheduler();

  dshow("here");
}

/********************************************VOID LOOP***********************************************/
void loop()
{
  //main loop is empty
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskRH(void *pvParameters)
{
  (void) pvParameters;  TickType_t xLastWakeTime;  int pin = 31;  pinMode(pin, OUTPUT); UBaseType_t uxHighWaterMark;
  xLastWakeTime = xTaskGetTickCount();
  bool value = false;


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    RightHip.FilterMovADC(600, 850, 45, -15);
    RightHip.GoToAngle(angleRH, RightKnee.GetAngle(), 150, 100, 60, 35, true);
    RightHip.Driver(RightHip.GetRotate(), true, RightHip.GetSpeed());

    /*debug task*/
    dshow(1);
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, tDelay);
  }
}

void TaskLH(void *pvParameters)
{
  (void) pvParameters;  TickType_t xLastWakeTime;  int pin = 49; UBaseType_t uxHighWaterMark;
  pinMode(pin, OUTPUT);
  bool value = false;
  xLastWakeTime = xTaskGetTickCount();


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    LeftHip.FilterMovADC(630, 880, 45, -15);
    LeftHip.GoToAngle(angleLH, LeftKnee.GetAngle(), 100, 90, 60, 35, true); // 150 100 60 35
    LeftHip.Driver(LeftHip.GetRotate(), true, LeftHip.GetSpeed());

    /*debug task*/
    dshow(2);
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, tDelay);
  }
}

void TaskRK(void *pvParameters)
{
  (void) pvParameters; int pin = 51; TickType_t xLastWakeTime; UBaseType_t uxHighWaterMark;
  pinMode(pin, OUTPUT);
  bool value = false;
  xLastWakeTime = xTaskGetTickCount();


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    RightKnee.FilterMovADC(320, 570, 45, -15);
    RightKnee.GoToAngle(angleRK, 0, 15, 15, 40, 35, false);
    RightKnee.Driver(RightKnee.GetRotate(), false, RightKnee.GetSpeed());

    /*debug task*/
    dshow(3);
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, tDelay);
  }
}

void TaskLK(void *pvParameters)
{
  (void) pvParameters;  TickType_t xLastWakeTime;  int pin = 53; UBaseType_t uxHighWaterMark;
  pinMode(pin, OUTPUT);
  bool value = false;
  xLastWakeTime = xTaskGetTickCount();


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    LeftKnee.FilterMovADC(290, 540, 45, -15);
    LeftKnee.GoToAngle(angleLK, 0, 15, 15, 35, 30, false); //15, 15, 40, 35, // 40, 40, 50, 30,
    LeftKnee.Driver(LeftKnee.GetRotate(), false, LeftKnee.GetSpeed());

    /*debug task*/
    dshow(4);
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);Serial.println();

    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, tDelay);
  }
}
