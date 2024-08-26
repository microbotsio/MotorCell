/*
 * Overview:
 * This code controls your motor's speed by setting the desired speed percentage using the MotorCell's Spin function.
 * The Spin function adjusts the motor's speed to the desired percentage of its maximum capability and returns the current RPM value.
 * It also prints the RPM value to the serial monitor for real-time monitoring.
 * Additionally, if the motor stalls, the function will automatically attempt to restart it, ensuring continuous operation.
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

  myMotorCell.Init(); /* Initialize the MotorCell*/
}

void loop() {
  uint16_t MotorRPM = myMotorCell.Spin(25); /* Set to 25% Speed Percentage of its maximum speed*/
}
