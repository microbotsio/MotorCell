/*
 * Overview:
 * This code implements a PID controller to regulate the motor speed to the desired RPM using the MotorCell's SpinPID function.
 * The SpinPID function continuously adjusts the motor's PWM to achieve and maintain the target RPM. The PID controller will 
 * dynamically compensates for disturbances and changes in load, ensuring smooth motor operation. This function will also 
 * automatically restart the motor if it stalls and will notify you via the serial monitor if the target speed cannot be reached.
 * Feel free to tweak the code with your own creative ideas!
 *
 * This example is specifically designed for use with a CodeCell or any other ESP32-based device.
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
  uint16_t MotorRPM = myMotorCell.SpinPID(20000); /* Set the target RPM to 30,000 using the PID controller */
}
