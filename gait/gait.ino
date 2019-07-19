#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include "motor_ibt.h"

/* Improve ADC sampling rate
   http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html
   https://www.instructables.com/id/Girino-Fast-Arduino-Oscilloscope/
*/

/* Human gait pattern
   https://www.researchgate.net/post/I_need_human_gait_data_for_normal_walk_and_fast_walk_is_there_any_standard_database_for_storing_this_type_of_data
*/

/* PWM frequency adjust
   http://www.robotrebels.org/index.php?topic=435.0
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
void TaskGait( void *pvParameters );

//float angleGait [5][10] = {
//  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//  {33.0, 29.9, 19.7, 7.5, -2.9, -8.2, -2.3, 16.7, 30.9, 32.8}, // RH
//  {7.9, 19.0, 16.2, 10.2, 7.3, 14.1, 37.2, 62.4, 54.2, 18.7},   // RK
//  {33.0, 29.9, 19.7, 7.5, -2.9, -8.2, -2.3, 16.7, 30.9, 32.8}, // LH
//  {7.9, 19.0, 16.2, 10.2, 7.3, 14.1, 37.2, 62.4, 54.2, 18.7}   // LK
//};StageNumber

//float angleGait [5][8] = {
//  {0, 0, 0, 0, 0, 0, 0, 0},
//  {30, 30, 5, -10, 0, 20, 30, 30},      // RH
//  {0, -15.0, -5, 0, -40, -60, -30, 0},   // RK
//  {30, 30, 5, -10, 0, 20, 30, 30},      // LH
//  {0, -15.0, -5, 0, -40, -60, -30, 0}   // LK
//};

#define MaxPhase 20
int angleGait[5][MaxPhase] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {33, 32, 29, 25, 19, 13, 7, 1, -2, -6, -8, -7, -2, 7, 16, 24, 30, 33, 32, 31},// RH
  {7, 13, 19, 18, 16, 13, 10, 7, 7, 9, 14, 22, 37, 53, 62, 62, 54, 38, 18, 4},// RK
  {33, 32, 29, 25, 19, 13, 7, 1, -2, -6, -8, -7, -2, 7, 16, 24, 30, 33, 32, 31},// LH
  {7, 13, 19, 18, 16, 13, 10, 7, 7, 9, 14, 22, 37, 53, 62, 62, 54, 38, 18, 4}// LK
};

volatile unsigned int phaseNow = 0; // increment for angleGait

double angleRH = 0; double angleRK = 0; double angleLH = 0; double angleLK = 0;

int tDelay = 2; // delay for motor task (in tick)
int gaitDelay = 1000;// delay for next gait (in ms)
#define pinSwitch 5
/********************************************VOID SETUP***********************************************/
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(pinSwitch, INPUT_PULLUP);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  //----------- Set PWM frequency for D11 & D12 --------------
  //  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz

  //----------- Set PWM frequency for D2, D3 & D5 ------------
  //  TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz
  //  TCCR3B = TCCR3B & B11111000 | B00000100; // set timer 3 divisor to 256 for PWM frequency of 122.55 Hz
  TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz

  //----------- Set PWM frequency for D6, D7 & D8 ------------
  //  TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz
  TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz

  //----------- Set PWM frequency for D9 & D10 ---------------
  //  TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz

  //  set adc prescaler to 16
  sbi(ADCSRA, ADPS2); // set bit
  cbi(ADCSRA, ADPS1); // clear bit
  cbi(ADCSRA, ADPS0);

  // Now set up many tasks to run independently.
  xTaskCreate(TaskRH,  (const portCHAR *)"RightHIP"  ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskLH,  (const portCHAR *)"LeftHIP"   ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskRK,  (const portCHAR *)"RightKNEE" ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskLK,  (const portCHAR *)"LeftKNEE"  ,  1000,  NULL,  1,  NULL );
  xTaskCreate(TaskGait,  (const portCHAR *)"Gait"  ,  192,  NULL,  2,  NULL );
  vTaskStartScheduler();//start scheduler

  pinMode(pinSwitch, INPUT_PULLUP);
}
/****************************************************************************************************/

/********************************************VOID LOOP***********************************************/
void loop()
{
  //main loop is empty
}
/****************************************************************************************************/


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskGait( void *pvParameters ) {
  (void) pvParameters;  TickType_t xLastWakeTime;  int pin = 31;  pinMode(pin, OUTPUT); UBaseType_t uxHighWaterMark;
  xLastWakeTime = xTaskGetTickCount();
  bool value = false;


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    //    phase = phase % 10;
    volatile int IsStop = digitalRead(pinSwitch);
    if (IsStop == HIGH) {
      angleRH = 0;
      angleRK = 0;
      angleLH = 0;
      angleLK = 0;
    } else {
      int offset = 10;
      float amp = -0.9;
      angleRH = amp * (angleGait[1][(phaseNow + 11) % MaxPhase] - 20);
      angleRK = amp * (angleGait[2][(phaseNow + 11) % MaxPhase] - 5);
      angleLH = amp * (angleGait[3][(phaseNow) % MaxPhase] - 20);
      angleLK = amp * (angleGait[4][(phaseNow) % MaxPhase] - 5);
    }

    RightHip.ResetPID();
    RightKnee.ResetPID();
    LeftHip.ResetPID();
    RightKnee.ResetPID();
    phaseNow++;

    /*debug task*/
    dprint(0);
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, (gaitDelay / portTICK_PERIOD_MS));
  }
}

void TaskRH(void *pvParameters)
{
  (void) pvParameters;  TickType_t xLastWakeTime;  int pin = 33;  pinMode(pin, OUTPUT); UBaseType_t uxHighWaterMark;
  xLastWakeTime = xTaskGetTickCount();
  bool value = false;


  for (;;)// A Task shall never return or exit.
  {
    digitalWrite(pin, value = !value);

    /*main task*/
    RightHip.FilterMovADC(580, 830, 45, -15);
    RightHip.GoToAngle(angleRH, RightKnee.GetAngle(), 90, 200, 60, 40, true);
    RightHip.Driver(RightHip.GetRotate(), true, RightHip.GetSpeed(), 35, 30);//30 25

    /*debug task*/
    dshow(1);
    //    dprint(RightHip.GetTarget());
    //    dprint(RightHip.GetAngle());
    //    dprint(RightHip.GetSpeed());
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*visualize gait*/
#if DEBUG == 1 || DEBUG == 5
    visualize(RightHip.GetTarget());
    visualize(RightHip.GetAngle());
    visualize(0);
    newline;
#endif

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
    LeftHip.GoToAngle(angleLH, LeftKnee.GetAngle(), 100, 150, 60, 35, true); // 150 100 60 35
    LeftHip.Driver(LeftHip.GetRotate(), true, LeftHip.GetSpeed(), 35, 40);

    /*debug task*/
    dshow(2);
    //    dprint(LeftHip.GetTarget());
    //    dprint(LeftHip.GetAngle());
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*visualize gait*/
#if DEBUG == 2 || DEBUG == 5
    visualize(1*bias + LeftHip.GetTarget());
    visualize(1*bias + LeftHip.GetAngle());
    visualize(1*bias);
    newline;
#endif
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
    RightKnee.GoToAngle(angleRK, 0, 42, 42, 40, 30, false);
    RightKnee.Driver(RightKnee.GetRotate(), false, RightKnee.GetSpeed(), 35, 35);

    /*debug task*/
    dshow(3);
    //        dprint(RightKnee.GetTarget());
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);

    /*visualize gait*/
#if DEBUG == 3 || DEBUG == 5
    visualize(2*bias + RightKnee.GetTarget());
    visualize(2*bias + RightKnee.GetAngle());
    visualize(2*bias);
    newline;
#endif
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
    LeftKnee.Driver(LeftKnee.GetRotate(), false, LeftKnee.GetSpeed(), 35, 35);

    /*debug task*/
    dshow(4);
    //        dprint(LeftKnee.GetTarget());
    //    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //    dprint(uxHighWaterMark);Serial.println();

    /*visualize gait*/
#if DEBUG == 4 || DEBUG == 5
    visualize(3*bias + LeftKnee.GetTarget());
    visualize(3*bias + LeftKnee.GetAngle());
    visualize(3*bias);
    newline;
#endif
#if DEBUG == 5
    Serial.println();
#endif
    /*delay*/
    //    vTaskDelay(tDelay);
    vTaskDelayUntil( &xLastWakeTime, tDelay);
  }
}
