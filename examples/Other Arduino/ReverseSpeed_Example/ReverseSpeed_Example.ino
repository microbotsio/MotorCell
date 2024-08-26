/*
 * Overview:
 * This example demonstrates how to reverse the motor's direction every 5 seconds and spin it at full speed using the MotorCell.
 * The MaxSpin function sets the motor to its maximum speed, while the ReverseSpin function changes the motor's direction of rotation.
 * The code alternates between clockwise and anticlockwise at maximum speed with a 5-second delay between direction changes.
 * Feel free to tweak the code with your own creative ideas!
 */

#include <MotorCell.h>

#define IN_pin1 2
#define OUT_pin2 3
#define FR_pin2 1

MotorCell myMotorCell(IN_pin1, OUT_pin2, FR_pin2); /*Configure the pins used for the MotorCell */

void setup() {
  Serial.begin(115200); /*Set up serial - Ensure Tools/USB_CDC_On_Boot is enabled for serial functionality */

  myMotorCell.Init(); /* Initialize the MotorCell */
}

void loop() {
  myMotorCell.MaxSpin();
  myMotorCell.ReverseSpin();
  delay(5000);
  myMotorCell.MaxSpin();
  myMotorCell.ReverseSpin();
  delay(5000);
}
