/*
 * Overview:
 * This code pulses 6 motors for 500ms each to create a spinning art project
 * Feel free to tweak the code with your own creative ideas!
 *
 */

#include <MotorCell.h>

#define IN_pin1 1
#define IN_pin2 2
#define IN_pin3 3
#define IN_pin5 5
#define IN_pin6 6
#define IN_pin7 7

#define pulse_ms 2000

int motor_num = 0;

MotorCell myMotorCell(1,2,3); 

void setup() {

}

void loop() {
  delay(100);
  motor_num++;
  switch (motor_num) {
    case 1:
       myMotorCell.Pulse(IN_pin1, pulse_ms);
      break;
    case 2:
       myMotorCell.Pulse(IN_pin2, pulse_ms);
      break;
    case 3:
       myMotorCell.Pulse(IN_pin3, pulse_ms);
      break;
    case 5:
       myMotorCell.Pulse(IN_pin5, pulse_ms);
      break;
    case 6:
       myMotorCell.Pulse(IN_pin6, pulse_ms);
      break;
    case 7:
       myMotorCell.Pulse(IN_pin7, pulse_ms);
      motor_num = 0;
      break;
  }
}
