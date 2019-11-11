#ifndef _DFRobotL298PShield_CONFIG_H_
#define _DFRobotL298PShield_CONFIG_H_

#include "motor_driver_config.h"

// Motor Driver Pins
//Left Motor
#define left_motor_in_1 5
#define left_motor_in_2 6
#define left_motor_pwm 9
//Right Motor
#define right_motor_in_1 7
#define right_motor_in_2 8
#define right_motor_pwm 10
// Left and Right motor driver objects

void setupMotors()
{
  pinMode(left_motor_in_1, OUTPUT);
  pinMode(left_motor_in_2, OUTPUT);
  pinMode(right_motor_in_1, OUTPUT);
  pinMode(right_motor_in_2, OUTPUT);
}
void commandLeftMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(left_motor_in_1, 0);
    digitalWrite(left_motor_in_2, 1);
  }
  else
  {
    digitalWrite(left_motor_in_1, 1);
    digitalWrite(left_motor_in_2, 0);
  }
  //  else if (cmd == 0)
  //  {
  //    digitalWrite(left_motor_in_1, 0);
  //    digitalWrite(left_motor_in_2, 0);
  //  }
  analogWrite(left_motor_pwm, abs(cmd));
}
void commandRightMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(right_motor_in_1, 1);
    digitalWrite(right_motor_in_2, 0);
  }
  else
  {
    digitalWrite(right_motor_in_1, 0);
    digitalWrite(right_motor_in_2, 1);
  }
//  else if (cmd == 0)
//  {
//    digitalWrite(right_motor_in_1, 0);
//    digitalWrite(right_motor_in_2, 0);
//  }
  analogWrite(right_motor_pwm, abs(cmd));
}


#endif  // _DFRobotL298PShield_CONFIG_H_
