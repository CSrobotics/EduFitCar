#ifndef _CS_ROBOTICS_H_
#define _CS_ROBOTICS_H_

#include "motor_driver_config.h"

// Motor Driver Pins
//Left Motor
#define M2_DIR 13
#define M2_SPEED 11
//Right Motor
#define M1_DIR 12
#define M1_SPEED 10
// Left and Right motor driver objects

void setupMotors()
{
  pinMode(M2_DIR, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
}
void commandLeftMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(M2_DIR, 1);
  }
  else
  {
    digitalWrite(M2_DIR, 0);
  }
  //  else if (cmd == 0)
  //  {
  //    digitalWrite(left_motor_in_1, 0);
  //    digitalWrite(left_motor_in_2, 0);
  //  }
  analogWrite(M2_SPEED, abs(cmd));
}
void commandRightMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(M1_DIR, 1);
  }
  else
  {
    digitalWrite(M1_DIR, 0);
  }
//  else if (cmd == 0)
//  {
//    digitalWrite(right_motor_in_1, 0);
//    digitalWrite(right_motor_in_2, 0);
//  }
  analogWrite(M1_SPEED, abs(cmd));
}


#endif  // _CS_ROBOTICS_H_
