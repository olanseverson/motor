#include "motor_ibt.h"
#include <Arduino_FreeRTOS.h>

// define Tasks
void TaskFilterADC( void *pvParameters );
void TaskDriver(void *pvParameters);
void TaskDebug( void *pvParameters );

Motor_IBT motor(5, 6, 14, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);
bool task1 = false;
bool task2 = false;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 115200 bits per second:
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two Tasks to run independently.
  bool x = xTaskCreate(
             TaskFilterADC
             ,  (const portCHAR *)"Filter ADC"  // A name just for humans
             ,  500  // This stack size can be checked & adjusted by reading the Stack Highwater
             ,  NULL
             ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
             ,  NULL );
  Serial.println(x);
  Serial.println("here");
  x = xTaskCreate(
        TaskDriver
        ,  (const portCHAR *) "Driver"
        ,  192  // Stack size
        ,  NULL
        ,  2  // Priority
        ,  NULL );
  Serial.println(x);
  Serial.println("heere");
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
  motor.GoToAngle(150, 20);
}

void loop()
{
  // Empty. Things are done in Tasks.
  //  long test = micros();
  ////  delay(1000);
  //  motor.FilterMovADC();
  //  Serial.print(motor.GetFilteredADC()); Serial.print("      "); Serial.println(motor.GetAngle());
  //  Serial.println(micros()-test);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskFilterADC( void *pvParameters)  // This is a Task.
/*  TaskFilterADC
    Filtering Potensiometer and convert to angle (position sensor)
*/
{
  TickType_t xLastWakeTime;

  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  for (;;) // A Task shall never return or exit.

  {
    digitalWrite(11, task1);
    task1 = !task1;

    motor.FilterMovADC();
    Serial.print(motor.GetFilteredADC()); Serial.print(" "); Serial.println(motor.GetAngle());

    //    static int t = 0;    t++; Serial.println(t);
    /* This task should execute every 15 milliseconds exactly. As per
      the vTaskDelay() function, time is measured in ticks, and the
      pdMS_TO_TICKS() macro is used to convert milliseconds into ticks.
      xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
      explicitly updated by the task. */
    //    vTaskDelayUntil( &xLastWakeTime, 2 );
    //    vTaskDelay(1);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //        Serial.println(uxHighWaterMark);
  }
}

void TaskDriver( void *pvParameters)  // This is a Task.
/*  TaskDriver
    Drive PWM to Motor(low level) and control using PID
*/
{
  TickType_t xLastWakeTime;
  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(12, task2);
    task2 = !task2;

    long x = micros();
    int pid = motor.PositionPID(0.8, 0.06, 0.8);
    motor.Driver(motor.GetRotate(), motor.GetSpeed() + pid);
    Serial.println(micros() - x);
    //    static int d = 0;
    //    d = d + 10; Serial.println(d);


    /* This task should execute every 15 milliseconds exactly. As per
      the vTaskDelay() function, time is measured in ticks, and the
      pdMS_TO_TICKS() macro is used to convert milliseconds into ticks.
      xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
      explicitly updated by the task. */
    //    vTaskDelayUntil( &xLastWakeTime, 2 );
    vTaskDelay(1);
  }
}
