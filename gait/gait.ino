#include "motor_ibt.h"

Motor_IBT motor(5, 6, 14, Serial); // IBT_Motor(int Pin_RPWM, int Pin_LPWM, int SensorPin);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 115200 bits per second:
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
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
