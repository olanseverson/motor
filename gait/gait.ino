#include "motor_ibt.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define Tasks
void TaskFilterADC( void *pvParameters );
void TaskDriver(void *pvParameters);
void TaskDebug( void *pvParameters );


Motor_IBT motor(5, 6, 14, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskFilterADC
    ,  (const portCHAR *)"Filter ADC"  // A name just for humans
    ,  1000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskDriver
    ,  (const portCHAR *) "Driver"
    ,  1000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
  motor.GoToAngle(150, 20);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskFilterADC( void *pvParameters __attribute__((unused)) )  // This is a Task.
/*  TaskFilterADC
    Filtering Potensiometer and convert to angle (position sensor)
*/
{
  TickType_t xLastWakeTime;
  for (;;) // A Task shall never return or exit.
  {
    motor.FilterMedADC();
    Serial.print(motor.GetFilteredADC());Serial.print(" "); Serial.println(motor.GetAngle());
    
    /* This task should execute every 15 milliseconds exactly. As per
      the vTaskDelay() function, time is measured in ticks, and the
      pdMS_TO_TICKS() macro is used to convert milliseconds into ticks.
      xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
      explicitly updated by the task. */
    vTaskDelayUntil( &xLastWakeTime, 1 );
  }
}

void TaskDriver( void *pvParameters __attribute__((unused)) )  // This is a Task.
/*  TaskDriver
    Drive PWM to Motor(low level) and control using PID
*/
{
  TickType_t xLastWakeTime;
  for (;;) // A Task shall never return or exit.
  {
    int pid = motor.PositionPID(0.8, 0.06, 0.8);
    Serial.print(" here ");
    motor.Driver(motor.GetRotate(), motor.GetSpeed() + pid);
    
    /* This task should execute every 15 milliseconds exactly. As per
      the vTaskDelay() function, time is measured in ticks, and the
      pdMS_TO_TICKS() macro is used to convert milliseconds into ticks.
      xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
      explicitly updated by the task. */
    vTaskDelayUntil( &xLastWakeTime, 1 );
  }
}
